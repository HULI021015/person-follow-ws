#include <algorithm>
#include <dlfcn.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "config.h"
#include "cuda_utils.h"
#include "grasp_planner.hpp"
#include "logging.h"
#include "postprocess.h"
#include "preprocess.h"
#include "types.h"

Logger gLogger;

namespace {

constexpr int kOutputSize =
    kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
constexpr int kOutputSegSize = 32 * (kInputH / 4) * (kInputW / 4);

std::string labelFor(
    const std::unordered_map<int, std::string> &labels, int class_id) {
  const auto it = labels.find(class_id);
  return it == labels.end() ? ("class_" + std::to_string(class_id)) : it->second;
}

bool finiteDetection(const Detection &det) {
  if (!std::isfinite(det.conf) || !std::isfinite(det.class_id)) {
    return false;
  }
  return std::all_of(
      std::begin(det.bbox), std::end(det.bbox),
      [](float value) { return std::isfinite(value); });
}

cv::Rect clippedRect(cv::Rect rect, const cv::Size &size) {
  return rect & cv::Rect(0, 0, size.width, size.height);
}

}  // namespace

class YoloV8SegNode final : public rclcpp::Node {
 public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<Image, Image>;

  YoloV8SegNode()
      : Node("graspgen_yolov8_seg") {
    declareParameters();
    loadParameters();
    loadLabels();
    initializeTensorRT();

    pointcloud_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("masked_pointcloud", 5);
    centroid_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "object_centroids", 5);
    debug_image_pub_ =
        create_publisher<Image>("detection_result", rclcpp::SensorDataQoS());
    planner_ = std::make_unique<graspgen_perception::GraspPlanner>(*this);

    camera_info_sub_ = create_subscription<CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&YoloV8SegNode::cameraInfoCallback, this,
                  std::placeholders::_1));

    color_sub_.subscribe(
        this, color_image_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(
        this, aligned_depth_topic_, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(8), color_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.08));
    sync_->registerCallback(
        std::bind(&YoloV8SegNode::imageCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
        get_logger(),
        "Ready. color=%s depth=%s info=%s frame=%s",
        color_image_topic_.c_str(), aligned_depth_topic_.c_str(),
        camera_info_topic_.c_str(), camera_frame_.c_str());
  }

  ~YoloV8SegNode() override {
    if (stream_) {
      cudaStreamSynchronize(stream_);
    }
    for (void *&buffer : device_buffers_) {
      if (buffer) {
        cudaFree(buffer);
        buffer = nullptr;
      }
    }
    if (stream_) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
    if (preprocess_initialized_) {
      cuda_preprocess_destroy();
    }
    if (show_gui_) {
      cv::destroyAllWindows();
    }
  }

 private:
  void declareParameters() {
    declare_parameter<std::string>("engine_path", "");
    declare_parameter<std::string>("labels_path", "");
    declare_parameter<std::string>(
        "color_image_topic", "/camera/color/image_raw");
    declare_parameter<std::string>(
        "aligned_depth_topic", "/camera/depth/image_raw");
    declare_parameter<std::string>(
        "camera_info_topic", "/camera/color/camera_info");
    declare_parameter<std::string>(
        "camera_frame", "camera_color_optical_frame");
    declare_parameter<double>("depth_scale", 0.001);
    declare_parameter<double>("confidence_threshold", 0.5);
    declare_parameter<double>("mask_threshold", 0.6);
    declare_parameter<double>("max_depth_m", 1.5);
    declare_parameter<int>("min_points", 80);
    declare_parameter<int>("point_stride", 2);
    declare_parameter<double>("gripper_max_opening", 0.10);
    declare_parameter<std::vector<std::string>>(
        "target_classes", {"bottle"});
    declare_parameter<bool>("publish_debug_image", true);
    declare_parameter<bool>("show_gui", false);
  }

  void loadParameters() {
    engine_path_ = get_parameter("engine_path").as_string();
    labels_path_ = get_parameter("labels_path").as_string();
    color_image_topic_ = get_parameter("color_image_topic").as_string();
    aligned_depth_topic_ = get_parameter("aligned_depth_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    depth_scale_ = get_parameter("depth_scale").as_double();
    confidence_threshold_ =
        get_parameter("confidence_threshold").as_double();
    mask_threshold_ = get_parameter("mask_threshold").as_double();
    max_depth_m_ = get_parameter("max_depth_m").as_double();
    min_points_ = std::max(1, static_cast<int>(get_parameter("min_points").as_int()));
    point_stride_ = std::max(1, static_cast<int>(get_parameter("point_stride").as_int()));
    gripper_max_opening_ =
        get_parameter("gripper_max_opening").as_double();
    publish_debug_image_ = get_parameter("publish_debug_image").as_bool();
    show_gui_ = get_parameter("show_gui").as_bool();
    const auto target_classes =
        get_parameter("target_classes").as_string_array();
    target_classes_.clear();
    for (const auto &name : target_classes) {
      if (!name.empty()) {
        target_classes_.insert(name);
      }
    }
    if (engine_path_.empty()) {
      throw std::runtime_error("engine_path is empty");
    }
    if (!(depth_scale_ > 0.0)) {
      throw std::runtime_error("depth_scale must be positive");
    }
  }

  void loadLabels() {
    std::ifstream input(labels_path_);
    if (!input) {
      RCLCPP_WARN(
          get_logger(), "Cannot open labels file: %s", labels_path_.c_str());
      return;
    }
    std::string line;
    int id = 0;
    while (std::getline(input, line)) {
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      labels_[id++] = line;
    }
  }

  void initializeTensorRT() {
    if (cudaSetDevice(kGpuId) != cudaSuccess) {
      throw std::runtime_error("cudaSetDevice failed");
    }

    // TensorRT built-in plugins
    initLibNvInferPlugins(&gLogger, "");

    // Load custom YOLO plugin used by yolov8n-seg.engine.
    // trtexec works only because we pass --plugins=libgraspgen_yolo_plugins.so;
    // the ROS node must load it explicitly before deserializing the engine.
    void *plugin_handle = dlopen("libgraspgen_yolo_plugins.so", RTLD_NOW | RTLD_GLOBAL);
    if (!plugin_handle) {
      RCLCPP_WARN(
          get_logger(),
          "dlopen libgraspgen_yolo_plugins.so failed: %s",
          dlerror());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded libgraspgen_yolo_plugins.so");
    }

    std::ifstream file(engine_path_, std::ios::binary | std::ios::ate);
    if (!file) {
      throw std::runtime_error("Cannot open TensorRT engine: " + engine_path_);
    }
    const auto end = file.tellg();
    if (end <= 0) {
      throw std::runtime_error("TensorRT engine is empty");
    }
    std::vector<char> data(static_cast<size_t>(end));
    file.seekg(0, std::ios::beg);
    file.read(data.data(), static_cast<std::streamsize>(data.size()));
    if (!file) {
      throw std::runtime_error("Failed to read complete TensorRT engine");
    }

    runtime_.reset(
        nvinfer1::createInferRuntime(gLogger),
        [](nvinfer1::IRuntime *ptr) {
          if (ptr) ptr->destroy();
        });
    if (!runtime_) {
      throw std::runtime_error("createInferRuntime failed");
    }
    engine_.reset(
        runtime_->deserializeCudaEngine(data.data(), data.size()),
        [](nvinfer1::ICudaEngine *ptr) {
          if (ptr) ptr->destroy();
        });
    if (!engine_) {
      throw std::runtime_error(
          "Engine deserialization failed. Rebuild it for this TensorRT version and GPU.");
    }
    context_.reset(
        engine_->createExecutionContext(),
        [](nvinfer1::IExecutionContext *ptr) {
          if (ptr) ptr->destroy();
        });
    if (!context_) {
      throw std::runtime_error("createExecutionContext failed");
    }
    if (engine_->getNbBindings() != 3) {
      throw std::runtime_error("Expected exactly 3 TensorRT bindings");
    }

    const int input_index = engine_->getBindingIndex(kInputTensorName);
    const int output_index = engine_->getBindingIndex(kOutputTensorName);
    const int proto_index = engine_->getBindingIndex("proto");
    if (input_index != 0 || output_index != 1 || proto_index != 2) {
      throw std::runtime_error(
          "Unexpected engine bindings; expected images/output/proto at 0/1/2");
    }

    const auto output_dims = engine_->getBindingDimensions(output_index);
    model_bboxes_ = output_dims.nbDims > 0 ? output_dims.d[0] : 0;
    if (model_bboxes_ <= 0) {
      throw std::runtime_error("Invalid output binding dimensions");
    }

    CUDA_CHECK(cudaStreamCreate(&stream_));
    CUDA_CHECK(cudaMalloc(
        &device_buffers_[0],
        kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc(
        &device_buffers_[1],
        kBatchSize * kOutputSize * sizeof(float)));
    CUDA_CHECK(cudaMalloc(
        &device_buffers_[2],
        kBatchSize * kOutputSegSize * sizeof(float)));
    output_host_.resize(kBatchSize * kOutputSize);
    proto_host_.resize(kBatchSize * kOutputSegSize);
    cuda_preprocess_init(kMaxInputImageSize);
    preprocess_initialized_ = true;
  }

  void cameraInfoCallback(const CameraInfo::ConstSharedPtr msg) {
    if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Ignoring invalid CameraInfo");
      return;
    }
    std::lock_guard<std::mutex> lock(camera_mutex_);
    camera_info_ = *msg;
    have_camera_info_ = true;
  }

  void imageCallback(
      const Image::ConstSharedPtr &color_msg,
      const Image::ConstSharedPtr &depth_msg) {
    const auto start = std::chrono::steady_clock::now();
    CameraInfo camera_info;
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (!have_camera_info_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Waiting for CameraInfo");
        return;
      }
      camera_info = camera_info_;
    }

    try {
      const cv::Mat color =
          cv_bridge::toCvShare(
              color_msg, sensor_msgs::image_encodings::BGR8)->image;
      const cv::Mat depth_raw = depthToCv(depth_msg);
      if (color.empty() || depth_raw.empty()) {
        return;
      }

      cv::Mat depth = depth_raw;
      if (color.size() != depth_raw.size()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Color (%dx%d) and depth (%dx%d) differ. Temporarily resizing depth "
            "to color size for debugging. For real grasping, enable proper D2C alignment.",
            color.cols, color.rows, depth_raw.cols, depth_raw.rows);
        cv::resize(depth_raw, depth, color.size(), 0, 0, cv::INTER_NEAREST);
      }
      if (camera_info.width != 0 &&
          (static_cast<int>(camera_info.width) != color.cols ||
           static_cast<int>(camera_info.height) != color.rows)) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "CameraInfo resolution does not match synchronized images");
        return;
      }

      std::vector<cv::Mat> batch{color.clone()};
      cuda_batch_preprocess(
          batch, static_cast<float *>(device_buffers_[0]),
          kInputW, kInputH, stream_);
      if (!context_->enqueue(
              kBatchSize, device_buffers_.data(), stream_, nullptr)) {
        throw std::runtime_error("TensorRT enqueue failed");
      }
      CUDA_CHECK(cudaMemcpyAsync(
          output_host_.data(), device_buffers_[1],
          output_host_.size() * sizeof(float),
          cudaMemcpyDeviceToHost, stream_));
      CUDA_CHECK(cudaMemcpyAsync(
          proto_host_.data(), device_buffers_[2],
          proto_host_.size() * sizeof(float),
          cudaMemcpyDeviceToHost, stream_));
      CUDA_CHECK(cudaStreamSynchronize(stream_));

      std::vector<std::vector<Detection>> result_batch;
      batch_nms(
          result_batch, output_host_.data(), 1, kOutputSize,
          static_cast<float>(confidence_threshold_), kNmsThresh);
      if (result_batch.empty()) {
        publishEmptyMarkers(color_msg->header);
        return;
      }

      auto &detections = result_batch.front();
      detections.erase(
          std::remove_if(
              detections.begin(), detections.end(),
              [this](const Detection &det) {
                return !finiteDetection(det) ||
                       det.conf < confidence_threshold_;
              }),
          detections.end());
      const auto masks = processMasks(proto_host_.data(), detections);
      cv::Mat debug = color.clone();
      processObjects(
          detections, masks, depth, color, camera_info, color_msg->header);
      drawDetections(debug, detections, masks);
      publishDebugImage(debug, color_msg->header);

      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start);
      RCLCPP_DEBUG(get_logger(), "Processing time: %ld ms", elapsed.count());
    } catch (const cv_bridge::Exception &error) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", error.what());
    } catch (const cv::Exception &error) {
      RCLCPP_ERROR(get_logger(), "OpenCV: %s", error.what());
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Frame processing failed: %s", error.what());
    }
  }

  cv::Mat depthToCv(const Image::ConstSharedPtr &msg) const {
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        msg->encoding == "mono16") {
      return cv_bridge::toCvShare(
                 msg, sensor_msgs::image_encodings::TYPE_16UC1)
          ->image;
    }
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      return cv_bridge::toCvShare(
                 msg, sensor_msgs::image_encodings::TYPE_32FC1)
          ->image;
    }
    throw std::runtime_error("Unsupported depth encoding: " + msg->encoding);
  }

  std::vector<cv::Mat> processMasks(
      const float *proto, const std::vector<Detection> &detections) const {
    std::vector<cv::Mat> masks;
    masks.reserve(detections.size());
    const int proto_height = kInputH / 4;
    const int proto_width = kInputW / 4;
    const int channel_size = proto_height * proto_width;

    for (const auto &det : detections) {
      cv::Mat mask = cv::Mat::zeros(
          proto_height, proto_width, CV_32FC1);
      const int left = static_cast<int>(std::floor(det.bbox[0] / 4.0f));
      const int top = static_cast<int>(std::floor(det.bbox[1] / 4.0f));
      const int right = static_cast<int>(
          std::ceil((det.bbox[0] + det.bbox[2]) / 4.0f));
      const int bottom = static_cast<int>(
          std::ceil((det.bbox[1] + det.bbox[3]) / 4.0f));
      const cv::Rect input_rect = clippedRect(
          cv::Rect(left, top, right - left, bottom - top), mask.size());
      if (input_rect.empty()) {
        masks.emplace_back();
        continue;
      }
      for (int y = input_rect.y; y < input_rect.br().y; ++y) {
        float *row = mask.ptr<float>(y);
        for (int x = input_rect.x; x < input_rect.br().x; ++x) {
          const int pixel = y * proto_width + x;
          float value = 0.0f;
          for (int channel = 0; channel < 32; ++channel) {
            value += det.mask[channel] *
                     proto[channel * channel_size + pixel];
          }
          row[x] = 1.0f / (1.0f + std::exp(-value));
        }
      }
      cv::Mat upsampled;
      cv::resize(mask, upsampled, cv::Size(kInputW, kInputH));
      masks.push_back(std::move(upsampled));
    }
    return masks;
  }

  cv::Mat scaleMask(const cv::Mat &mask, const cv::Size &target) const {
    if (mask.empty()) {
      return {};
    }
    const float width_ratio =
        static_cast<float>(kInputW) / static_cast<float>(target.width);
    const float height_ratio =
        static_cast<float>(kInputH) / static_cast<float>(target.height);
    int x = 0;
    int y = 0;
    int width = kInputW;
    int height = kInputH;
    if (height_ratio > width_ratio) {
      height = std::max(
          1, static_cast<int>(std::round(width_ratio * target.height)));
      y = (kInputH - height) / 2;
    } else {
      width = std::max(
          1, static_cast<int>(std::round(height_ratio * target.width)));
      x = (kInputW - width) / 2;
    }
    const cv::Rect roi =
        clippedRect(cv::Rect(x, y, width, height), mask.size());
    if (roi.empty()) {
      return {};
    }
    cv::Mat result;
    cv::resize(mask(roi), result, target, 0.0, 0.0, cv::INTER_LINEAR);
    return result;
  }

  void processObjects(
      const std::vector<Detection> &detections,
      const std::vector<cv::Mat> &masks,
      const cv::Mat &depth,
      const cv::Mat &color,
      const CameraInfo &camera_info,
      const std_msgs::msg::Header &source_header) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = source_header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    auto merged =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr best_cloud;
    size_t best_size = 0;
    std_msgs::msg::Header header = source_header;
    header.frame_id =
        camera_frame_.empty() ? source_header.frame_id : camera_frame_;

    const float fx = static_cast<float>(camera_info.k[0]);
    const float fy = static_cast<float>(camera_info.k[4]);
    const float cx = static_cast<float>(camera_info.k[2]);
    const float cy = static_cast<float>(camera_info.k[5]);

    for (size_t index = 0; index < detections.size(); ++index) {
      if (index >= masks.size()) {
        break;
      }
      const int class_id =
          static_cast<int>(std::lround(detections[index].class_id));
      const std::string label = labelFor(labels_, class_id);
      if (!target_classes_.empty() &&
          target_classes_.find(label) == target_classes_.end()) {
        continue;
      }

      const cv::Mat mask = scaleMask(masks[index], color.size());
      if (mask.empty() || mask.size() != depth.size()) {
        continue;
      }
      float bbox[4];
      std::copy(
          std::begin(detections[index].bbox),
          std::end(detections[index].bbox), bbox);
      const cv::Rect roi = clippedRect(get_rect(
          const_cast<cv::Mat &>(color), bbox), color.size());
      if (roi.empty()) {
        continue;
      }

      auto cloud =
          std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      cloud->reserve(
          static_cast<size_t>(roi.area()) /
          static_cast<size_t>(point_stride_ * point_stride_));
      for (int v = roi.y; v < roi.br().y; v += point_stride_) {
        const float *mask_row = mask.ptr<float>(v);
        for (int u = roi.x; u < roi.br().x; u += point_stride_) {
          if (mask_row[u] <= mask_threshold_) {
            continue;
          }
          float z = 0.0f;
          if (depth.type() == CV_16UC1) {
            z = static_cast<float>(depth.at<uint16_t>(v, u) * depth_scale_);
          } else if (depth.type() == CV_32FC1) {
            z = depth.at<float>(v, u);
          }
          if (!std::isfinite(z) || z <= 0.0f || z > max_depth_m_) {
            continue;
          }

          const cv::Vec3b bgr = color.at<cv::Vec3b>(v, u);
          pcl::PointXYZRGB point;
          point.x = (static_cast<float>(u) - cx) * z / fx;
          point.y = (static_cast<float>(v) - cy) * z / fy;
          point.z = z;
          point.r = bgr[2];
          point.g = bgr[1];
          point.b = bgr[0];
          cloud->push_back(point);
        }
      }
      if (cloud->size() < static_cast<size_t>(min_points_)) {
        continue;
      }

      Eigen::Vector4f centroid;
      if (pcl::compute3DCentroid(*cloud, centroid) == 0 ||
          !centroid.allFinite()) {
        continue;
      }
      appendCentroidMarkers(
          markers, header, index, label, detections[index].conf, centroid);
      *merged += *cloud;
      if (cloud->size() > best_size) {
        best_size = cloud->size();
        best_cloud = cloud;
      }
    }

    centroid_pub_->publish(markers);
    if (!merged->empty()) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*merged, cloud_msg);
      cloud_msg.header = header;
      pointcloud_pub_->publish(cloud_msg);
    }
    if (best_cloud) {
      geometry_msgs::msg::PoseStamped grasp_pose;
      planner_->compute(
          best_cloud, header, static_cast<float>(gripper_max_opening_),
          grasp_pose);
    }
  }

  void appendCentroidMarkers(
      visualization_msgs::msg::MarkerArray &array,
      const std_msgs::msg::Header &header,
      size_t index,
      const std::string &label,
      float confidence,
      const Eigen::Vector4f &centroid) const {
    visualization_msgs::msg::Marker sphere;
    sphere.header = header;
    sphere.ns = "centroids";
    sphere.id = static_cast<int>(index);
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = centroid.x();
    sphere.pose.position.y = centroid.y();
    sphere.pose.position.z = centroid.z();
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04;
    sphere.color.r = 1.0f;
    sphere.color.a = 1.0f;
    sphere.lifetime.sec = 0;
    sphere.lifetime.nanosec = 500000000;
    array.markers.push_back(sphere);

    auto text = sphere;
    text.id += 1000;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.text = label + " " + cv::format("%.2f", confidence);
    text.pose.position.z += 0.08;
    text.scale.z = 0.04;
    text.color.r = text.color.g = text.color.b = 1.0f;
    array.markers.push_back(text);
  }

  void publishEmptyMarkers(const std_msgs::msg::Header &header) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    centroid_pub_->publish(markers);
  }

  void drawDetections(
      cv::Mat &image,
      const std::vector<Detection> &detections,
      const std::vector<cv::Mat> &masks) const {
    static const std::array<cv::Scalar, 6> colors{
        cv::Scalar(56, 56, 255), cv::Scalar(151, 157, 255),
        cv::Scalar(31, 112, 255), cv::Scalar(29, 178, 255),
        cv::Scalar(49, 210, 207), cv::Scalar(10, 249, 72)};
    for (size_t index = 0;
         index < detections.size() && index < masks.size(); ++index) {
      const cv::Mat mask = scaleMask(masks[index], image.size());
      if (mask.empty()) {
        continue;
      }
      float bbox[4];
      std::copy(
          std::begin(detections[index].bbox),
          std::end(detections[index].bbox), bbox);
      const cv::Rect roi = clippedRect(
          get_rect(image, bbox), image.size());
      if (roi.empty()) {
        continue;
      }
      const int class_id =
          static_cast<int>(std::lround(detections[index].class_id));
      const cv::Scalar color =
          colors[static_cast<size_t>(std::abs(class_id)) % colors.size()];
      cv::Mat binary;
      cv::threshold(mask(roi), binary, mask_threshold_, 255, cv::THRESH_BINARY);
      binary.convertTo(binary, CV_8UC1);
      cv::Mat overlay(roi.size(), image.type(), color);
      cv::addWeighted(image(roi), 0.55, overlay, 0.45, 0.0, overlay);
      overlay.copyTo(image(roi), binary);
      cv::rectangle(image, roi, color, 2);
      cv::putText(
          image,
          labelFor(labels_, class_id) + cv::format(
              " %.2f", detections[index].conf),
          cv::Point(roi.x, std::max(15, roi.y - 4)),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }
  }

  void publishDebugImage(
      const cv::Mat &image, const std_msgs::msg::Header &header) {
    if (publish_debug_image_) {
      debug_image_pub_->publish(
          *cv_bridge::CvImage(
               header, sensor_msgs::image_encodings::BGR8, image)
               .toImageMsg());
    }
    if (show_gui_) {
      cv::imshow("GraspGen ROS2", image);
      cv::waitKey(1);
    }
  }

  std::string engine_path_;
  std::string labels_path_;
  std::string color_image_topic_;
  std::string aligned_depth_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_;
  double depth_scale_{0.001};
  double confidence_threshold_{0.5};
  double mask_threshold_{0.6};
  double max_depth_m_{1.5};
  int min_points_{80};
  int point_stride_{2};
  double gripper_max_opening_{0.10};
  bool publish_debug_image_{true};
  bool show_gui_{false};
  std::unordered_set<std::string> target_classes_;
  std::unordered_map<int, std::string> labels_;

  message_filters::Subscriber<Image> color_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      centroid_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_image_pub_;
  std::unique_ptr<graspgen_perception::GraspPlanner> planner_;

  std::mutex camera_mutex_;
  CameraInfo camera_info_;
  bool have_camera_info_{false};

  std::shared_ptr<nvinfer1::IRuntime> runtime_;
  std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_{nullptr};
  std::array<void *, 3> device_buffers_{nullptr, nullptr, nullptr};
  std::vector<float> output_host_;
  std::vector<float> proto_host_;
  int model_bboxes_{0};
  bool preprocess_initialized_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<YoloV8SegNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(
        rclcpp::get_logger("graspgen_yolov8_seg"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
