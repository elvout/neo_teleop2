#include "ur_teleop_node.hpp"

#include <cmath>
#include <numbers>
#include <tuple>

#include <Eigen/Geometry>
#include <moveit_servo/servo.hpp>

namespace {
Vector6d TransformTwist(const Vector6d& twist, const Eigen::Isometry3d& T_new_old) {
  Vector6d new_twist = Vector6d::Zero();

  new_twist.segment(0, 3) = T_new_old.rotation() * twist.segment(0, 3);
  new_twist.segment(3, 3) = T_new_old.rotation() * twist.segment(3, 3);

  return new_twist;
}
}  // namespace

void URTeleopNode::switch_controllers(std::string_view activate, std::string_view deactivate) {
  const auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers.emplace_back(activate);
  request->deactivate_controllers.emplace_back(deactivate);
  request->activate_asap = true;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  request->timeout.sec = 1;
  std::ignore = switch_controller_client_->async_send_request(request);
}

void URTeleopNode::send_cmd() {
  if (control_mode_ == URTeleopControlMode::kNotControlling) {
    return;
  }

  if (control_mode_ == URTeleopControlMode::kFreedrive) {
    auto bool_msg = std_msgs::msg::Bool();
    bool_msg.data = true;
    freedrive_keepalive_pub_->publish(bool_msg);
    return;
  }

  Vector6d next_command_ee_twist = servo_state_.target_ee_twist;

  const Eigen::Vector3d next_linear_vel_diff =
      next_command_ee_twist.head(3) - servo_state_.prev_commanded_ee_twist.head(3);
  const Eigen::Vector3d next_linear_accel = next_linear_vel_diff * update_frequency_;
  if (next_linear_accel.norm() > servo_max_linear_accel_) {
    next_command_ee_twist.head(3) =
        servo_state_.prev_commanded_ee_twist.head(3) +
        next_linear_vel_diff * servo_max_linear_accel_ / next_linear_accel.norm();
  }

  const Eigen::Vector3d next_angular_vel_diff =
      next_command_ee_twist.tail(3) - servo_state_.prev_commanded_ee_twist.tail(3);
  const Eigen::Vector3d next_angular_accel = next_angular_vel_diff * update_frequency_;
  if (next_angular_accel.norm() > servo_max_angular_accel_) {
    next_command_ee_twist.tail(3) =
        servo_state_.prev_commanded_ee_twist.tail(3) +
        next_angular_vel_diff * servo_max_angular_accel_ / next_angular_accel.norm();
  }

  servo_state_.prev_commanded_ee_twist = next_command_ee_twist;

  if (control_mode_ == URTeleopControlMode::kWholeBody) {
    auto bool_msg = std_msgs::msg::Bool();
    bool_msg.data = true;
    wholebody_keepalive_pub_->publish(bool_msg);

    geometry_msgs::msg::TwistStamped desired_twist_msg;
    desired_twist_msg.header.stamp = now();
    desired_twist_msg.header.frame_id = "vg10_grasp_center";
    desired_twist_msg.twist.linear.x = next_command_ee_twist(0);
    desired_twist_msg.twist.linear.y = next_command_ee_twist(1);
    desired_twist_msg.twist.linear.z = next_command_ee_twist(2);
    desired_twist_msg.twist.angular.x = next_command_ee_twist(3);
    desired_twist_msg.twist.angular.y = next_command_ee_twist(4);
    desired_twist_msg.twist.angular.z = next_command_ee_twist(5);
    wholebody_desired_twist_pub_->publish(desired_twist_msg);

    return;
  }

  if (control_mode_ == URTeleopControlMode::kServo) {
    const moveit::core::RobotStatePtr robot_state =
        planning_scene_monitor_->getStateMonitor()->getCurrentState();

    // convert the twist of the EE point from EE frame to base link frame
    const Eigen::Isometry3d& T_baselink_ee =
        robot_state->getGlobalLinkTransform("vg10_grasp_center");

    next_command_ee_twist = TransformTwist(next_command_ee_twist, T_baselink_ee);

    if (base_mode_ == ServoBaseMovementMode::kFull) {
      // split into components: base takes vx, vy, omegaz
      Vector6d next_command_base_twist = Vector6d::Zero();
      next_command_base_twist[5] = next_command_ee_twist[5];

      const double phi =
          std::atan2(T_baselink_ee.translation().y(), T_baselink_ee.translation().x());
      const double disp =
          Eigen::Vector2d{T_baselink_ee.translation().x(), T_baselink_ee.translation().y()}.norm();
      next_command_base_twist.segment(0, 2) =
          next_command_ee_twist.segment(0, 2) -
          Eigen::Rotation2Dd(phi + std::numbers::pi / 2.0) *
              (disp * next_command_base_twist[5] * Eigen::Vector2d::UnitX());

      next_command_ee_twist[0] = 0.0;
      next_command_ee_twist[1] = 0.0;
      next_command_ee_twist[5] = 0.0;
      geometry_msgs::msg::Twist base_twist_command;
      base_twist_command.linear.x = next_command_base_twist[0];
      base_twist_command.linear.y = next_command_base_twist[1];
      base_twist_command.angular.z = next_command_base_twist[5];
      base_twist_pub_->publish(base_twist_command);
    }

    const moveit_servo::TwistCommand command{"base_link", next_command_ee_twist};
    const moveit_servo::KinematicState next_state = servo_->getNextJointState(robot_state, command);

    const auto forward_vel_command =
        moveit_servo::composeMultiArrayMessage(servo_->getParams(), next_state);
    servo_control_pub_->publish(forward_vel_command);
  }
}
