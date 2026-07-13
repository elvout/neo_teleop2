#include "ur_teleop_node.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_servo/servo.hpp>

void URTeleopNode::setup_moveit_servo() {
  const std::string servo_param_namespace = "moveit_servo";
  const auto servo_param_listener =
      std::make_shared<const servo::ParamListener>(this->shared_from_this(), servo_param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  planning_scene_monitor_ =
      moveit_servo::createPlanningSceneMonitor(this->shared_from_this(), servo_params);
  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startStateMonitor();

  servo_ = std::make_shared<moveit_servo::Servo>(this->shared_from_this(), servo_param_listener,
                                                 planning_scene_monitor_);
  servo_->setCommandType(moveit_servo::CommandType::TWIST);

  servo_control_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(servo_params.command_out_topic, 1);

  // This node may launch before the MoveIt node, so we may need to wait for its
  // move_group action server to become available. We retry initialization
  // asynchronously so the rest of this node is not blocked.
  const std::weak_ptr<rclcpp::Node> weak_node = this->weak_from_this();
  move_group_setup_thread_ = std::jthread([this, weak_node](std::stop_token stop_token) {
    constexpr std::chrono::milliseconds kRetrySleep(1000);
    constexpr std::chrono::duration<double> kActionServerTimeout(0.25);

    while (rclcpp::ok() && !stop_token.stop_requested()) {
      try {
        const auto node = weak_node.lock();
        if (!node) {
          return;
        }

        auto move_group_interface =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                node, reset_pose_params_.move_group_name, std::shared_ptr<tf2_ros::Buffer>(),
                rclcpp::Duration::from_seconds(kActionServerTimeout.count()));

        if (!move_group_interface->getMoveGroupClient().wait_for_action_server(
                kActionServerTimeout)) {
          throw std::runtime_error("Timed out waiting for move_group action server");
        }

        move_group_interface->setPlanningTime(reset_pose_params_.planning_time_sec);
        move_group_interface->setMaxVelocityScalingFactor(
            std::clamp(reset_pose_params_.max_velocity_scaling, 0.0, 1.0));
        move_group_interface->setMaxAccelerationScalingFactor(
            std::clamp(reset_pose_params_.max_acceleration_scaling, 0.0, 1.0));

        move_group_interface_.store(move_group_interface);

        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for group '%s'.",
                    reset_pose_params_.move_group_name.c_str());
        return;
      } catch (const std::exception& ex) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "MoveGroupInterface initialization failed for group '"
                                        << reset_pose_params_.move_group_name << "': " << ex.what()
                                        << ". Retrying...");
      }

      std::this_thread::sleep_for(kRetrySleep);
    }
  });
}

std::expected<void, std::string> URTeleopNode::reset_pose() {
  const auto move_group_interface = move_group_interface_.load();

  if (!move_group_interface) {
    return std::unexpected(
        "Pose reset unavailable: MoveGroupInterface is not initialized. Is move_group "
        "running?");
  }

  if (control_mode_ != URTeleopControlMode::kNotControlling) {
    return std::unexpected("Pose reset rejected: teleop is active. Release deadman buttons first.");
  }

  move_group_interface->setStartStateToCurrentState();
  const std::vector<double> joint_pos_target = {1.92, -1.54952, -2.13627, -2.59727, -1.21781, 0.0};
  if (!move_group_interface->setJointValueTarget(joint_pos_target)) {
    return std::unexpected("Pose reset target contains invalid joint values for move group '" +
                           reset_pose_params_.move_group_name + "'.");
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(move_group_interface->plan(plan))) {
    return std::unexpected("Planning failed for pose reset.");
  }
  if (!static_cast<bool>(move_group_interface->execute(plan))) {
    return std::unexpected("Execution failed for pose reset.");
  }

  return {};
}
