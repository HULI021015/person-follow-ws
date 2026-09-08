#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigenvalues>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace graspgen_perception {

class GraspPlanner {
 public:
  explicit GraspPlanner(rclcpp::Node &node)
      : node_(node),
        aabb_pub_(node.create_publisher<visualization_msgs::msg::Marker>(
            "aabb_marker", 1)),
        axes_pub_(node.create_publisher<visualization_msgs::msg::Marker>(
            "grasp_pose_marker", 1)),
        pose_pub_(node.create_publisher<geometry_msgs::msg::PoseStamped>(
            "grasp_pose", 1)) {}

  bool compute(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
      const std_msgs::msg::Header &header,
      float gripper_max_opening,
      geometry_msgs::msg::PoseStamped &pose) {
    if (!cloud || cloud->size() < 20) {
      return false;
    }

    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(std::min<int>(30, static_cast<int>(cloud->size()) - 1));
    sor.setStddevMulThresh(1.5);
    sor.filter(*filtered);
    if (filtered->size() < 20) {
      return false;
    }

    Eigen::Vector4f centroid4;
    Eigen::Matrix3f covariance;
    pcl::compute3DCentroid(*filtered, centroid4);
    pcl::computeCovarianceMatrixNormalized(*filtered, centroid4, covariance);
    if (!covariance.allFinite()) {
      return false;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success) {
      return false;
    }

    Eigen::Matrix3f basis;
    basis.col(0) = solver.eigenvectors().col(2).normalized();
    basis.col(1) = solver.eigenvectors().col(1).normalized();
    basis.col(2) = basis.col(0).cross(basis.col(1)).normalized();
    basis.col(1) = basis.col(2).cross(basis.col(0)).normalized();

    Eigen::Matrix4f to_pca = Eigen::Matrix4f::Identity();
    to_pca.block<3, 3>(0, 0) = basis.transpose();
    to_pca.block<3, 1>(0, 3) =
        -basis.transpose() * centroid4.head<3>();

    pcl::PointCloud<pcl::PointXYZRGB> local;
    pcl::transformPointCloud(*filtered, local, to_pca);
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D(local, min_pt, max_pt);

    const Eigen::Vector3f dimensions(
        max_pt.x - min_pt.x,
        max_pt.y - min_pt.y,
        max_pt.z - min_pt.z);
    Eigen::Index grasp_axis = 0;
    const float min_dimension = dimensions.minCoeff(&grasp_axis);
    if (!std::isfinite(min_dimension) || min_dimension > gripper_max_opening) {
      return false;
    }

    const Eigen::Vector3f center_local(
        (min_pt.x + max_pt.x) * 0.5f,
        (min_pt.y + max_pt.y) * 0.5f,
        (min_pt.z + max_pt.z) * 0.5f);
    const Eigen::Vector3f center = basis * center_local + centroid4.head<3>();

    Eigen::Matrix3f rotation;
    rotation.col(2) = basis.col(grasp_axis).normalized();
    Eigen::Vector3f approach(0.0f, 0.0f, -1.0f);
    if (std::abs(rotation.col(2).dot(approach)) > 0.95f) {
      approach = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    }
    rotation.col(0) =
        (approach - approach.dot(rotation.col(2)) * rotation.col(2))
            .normalized();
    rotation.col(1) = rotation.col(2).cross(rotation.col(0)).normalized();
    if (rotation.determinant() < 0.0f) {
      rotation.col(1) *= -1.0f;
    }

    const Eigen::Quaternionf q(rotation);
    pose.header = header;
    pose.pose.position.x = center.x();
    pose.pose.position.y = center.y();
    pose.pose.position.z = center.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);

    publishMarkers(pose, dimensions, basis, center);
    return true;
  }

 private:
  void publishMarkers(
      const geometry_msgs::msg::PoseStamped &pose,
      const Eigen::Vector3f &dimensions,
      const Eigen::Matrix3f &basis,
      const Eigen::Vector3f &center) {
    visualization_msgs::msg::Marker box;
    box.header = pose.header;
    box.ns = "grasp_aabb";
    box.id = 0;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.position.x = center.x();
    box.pose.position.y = center.y();
    box.pose.position.z = center.z();
    const Eigen::Quaternionf box_q(basis);
    box.pose.orientation.x = box_q.x();
    box.pose.orientation.y = box_q.y();
    box.pose.orientation.z = box_q.z();
    box.pose.orientation.w = box_q.w();
    box.scale.x = std::max(0.001f, dimensions.x());
    box.scale.y = std::max(0.001f, dimensions.y());
    box.scale.z = std::max(0.001f, dimensions.z());
    box.color.r = 0.1f;
    box.color.g = 0.8f;
    box.color.b = 0.2f;
    box.color.a = 0.25f;
    aabb_pub_->publish(box);

    visualization_msgs::msg::Marker axes;
    axes.header = pose.header;
    axes.ns = "grasp_pose";
    axes.id = 0;
    axes.type = visualization_msgs::msg::Marker::ARROW;
    axes.action = visualization_msgs::msg::Marker::ADD;
    axes.pose = pose.pose;
    axes.scale.x = 0.12;
    axes.scale.y = 0.018;
    axes.scale.z = 0.018;
    axes.color.r = 1.0f;
    axes.color.g = 0.2f;
    axes.color.a = 1.0f;
    axes_pub_->publish(axes);
  }

  rclcpp::Node &node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr aabb_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr axes_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

}  // namespace graspgen_perception
