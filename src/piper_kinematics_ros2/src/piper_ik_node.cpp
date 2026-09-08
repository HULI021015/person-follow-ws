#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "piper_jacobian_ik.hpp"

class PiperIkNode final : public rclcpp::Node {
 public:
  PiperIkNode()
      : Node("piper_ik"),
        solver_(PiperForwardKinematics::MODIFIED),
        tf_broadcaster_(*this) {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    solver_.setMaxIterations(declare_parameter<int>("max_iterations", 100));
    solver_.setPositionTolerance(
        declare_parameter<double>("position_tolerance", 0.002));
    solver_.setOrientationTolerance(
        declare_parameter<double>("orientation_tolerance", 0.03));
    solver_.setDampingFactor(
        declare_parameter<double>("damping_factor", 0.10));
    max_joint_step_ = declare_parameter<double>("max_joint_step", 1.0);
    enforce_joint_step_limit_ =
        declare_parameter<bool>("enforce_joint_step_limit", false);
    validate_joint_limits_ =
        declare_parameter<bool>("validate_joint_limits", false);
    default_speed_percent_ =
        declare_parameter<double>("default_speed_percent", 30.0);

    joint_limits_ = {{
        {-2.618, 2.618},
        {0.0, M_PI},
        {-M_PI, 0.0},
        {-2.967, 2.967},
        {-1.2, 1.2},
        {-1.22, 1.22},
    }};
    solver_.setJointLimits(
        std::vector<std::pair<double, double>>(
            joint_limits_.begin(), joint_limits_.end()));

    command_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("/joint_ctrl_cmd", 5);
    status_pub_ = create_publisher<std_msgs::msg::Bool>("/ik_status", 5);
    feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(
            &PiperIkNode::feedbackCallback, this, std::placeholders::_1));
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 5,
        std::bind(
            &PiperIkNode::targetCallback, this, std::placeholders::_1));
    gripper_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/gripper_cmd", 5,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::array<double, 6> joints{};
          bool ready = false;
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            gripper_opening_ = std::clamp(msg->data, 0.0, 0.07);
            joints = current_joints_;
            ready = have_feedback_;
          }
          if (ready) {
            publishCommand(
                std::vector<double>(joints.begin(), joints.end()),
                gripper_opening_);
          }
        });
  }

 private:
  void feedbackCallback(
      const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::array<double, 6> ordered{};
    std::array<bool, 6> found{};
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      for (size_t joint = 0; joint < 6; ++joint) {
        if (msg->name[i] == "joint" + std::to_string(joint + 1)) {
          ordered[joint] = msg->position[i];
          found[joint] = true;
        }
      }
    }
    if (!std::all_of(found.begin(), found.end(), [](bool value) {
          return value;
        })) {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints_ = ordered;
    have_feedback_ = true;
  }

  void targetCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std_msgs::msg::Bool status;
    status.data = false;
    try {
      if (msg->header.frame_id != base_frame_) {
        throw std::runtime_error(
            "target_pose must already be transformed into " + base_frame_);
      }
      const auto &orientation = msg->pose.orientation;
      Eigen::Quaterniond quaternion(
          orientation.w, orientation.x, orientation.y, orientation.z);
      if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-6) {
        throw std::runtime_error("target quaternion is invalid");
      }
      quaternion.normalize();

      Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
      target.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
      target(0, 3) = msg->pose.position.x;
      target(1, 3) = msg->pose.position.y;
      target(2, 3) = msg->pose.position.z;
      if (!target.allFinite()) {
        throw std::runtime_error("target pose contains NaN/Inf");
      }

      std::array<double, 6> seed{};
      double gripper = 0.0;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!have_feedback_) {
          throw std::runtime_error("waiting for real joint feedback");
        }
        seed = current_joints_;
        gripper = gripper_opening_;
      }

      Eigen::VectorXd final_error;
      const std::vector<double> solution = solver_.computeIK(
          std::vector<double>(seed.begin(), seed.end()), target,
          false, &final_error);
      validateSolution(seed, solution);
      publishCommand(solution, gripper);
      publishTargetTf(*msg);
      status.data = true;
      RCLCPP_INFO(
          get_logger(), "IK accepted: position error %.2f mm, orientation %.3f rad",
          final_error.head<3>().norm() * 1000.0,
          final_error.tail<3>().norm());
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "IK rejected: %s", error.what());
    }
    status_pub_->publish(status);
  }

  void validateSolution(
      const std::array<double, 6> &seed,
      const std::vector<double> &solution) const {
    if (solution.size() != 6) {
      throw std::runtime_error("solver returned wrong joint count");
    }
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(solution[i])) {
        throw std::runtime_error("solver returned NaN/Inf");
      }
      if (validate_joint_limits_ &&
          (solution[i] < joint_limits_[i].first ||
           solution[i] > joint_limits_[i].second)) {
        throw std::runtime_error("solver exceeded joint limits");
      }
      if (enforce_joint_step_limit_ && max_joint_step_ > 0.0 &&
          std::abs(solution[i] - seed[i]) > max_joint_step_) {
        throw std::runtime_error(
            "joint step exceeds max_joint_step; rejecting unsafe jump");
      }
    }
  }

  void publishCommand(
      const std::vector<double> &solution, double gripper) {
    sensor_msgs::msg::JointState command;
    command.header.stamp = now();
    command.name = {
        "joint1", "joint2", "joint3", "joint4",
        "joint5", "joint6", "gripper"};
    command.position = solution;
    command.position.push_back(gripper);
    command.velocity.assign(7, default_speed_percent_);
    command.effort.assign(7, 1.0);
    command_pub_->publish(command);
  }

  void publishTargetTf(
      const geometry_msgs::msg::PoseStamped &target) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = target.header;
    transform.header.stamp = now();
    transform.child_frame_id = "ik_target";
    transform.transform.translation.x = target.pose.position.x;
    transform.transform.translation.y = target.pose.position.y;
    transform.transform.translation.z = target.pose.position.z;
    transform.transform.rotation = target.pose.orientation;
    tf_broadcaster_.sendTransform(transform);
  }

  PiperJacobianIK solver_;
  std::array<std::pair<double, double>, 6> joint_limits_;
  std::string base_frame_;
  double max_joint_step_{1.0};
  bool enforce_joint_step_limit_{false};
  bool validate_joint_limits_{false};
  double default_speed_percent_{30.0};

  std::mutex state_mutex_;
  std::array<double, 6> current_joints_{};
  bool have_feedback_{false};
  double gripper_opening_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PiperIkNode>());
  rclcpp::shutdown();
  return 0;
}
