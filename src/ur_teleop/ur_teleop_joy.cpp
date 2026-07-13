#include "ur_teleop_node.hpp"

#include <tuple>

bool URTeleopNode::is_button_pressed(std::span<const int32_t> buttons,
                                     const int button_index) const {
  if (button_index < 0 || button_index >= static_cast<int>(buttons.size())) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Invalid button index " << button_index << ". Joystick has "
                                                         << buttons.size() << " buttons.");
    return false;
  }
  return static_cast<bool>(buttons[button_index]);
}

float URTeleopNode::get_axis_value(std::span<const float> axes, const int axis_index) const {
  if (axis_index < 0 || axis_index >= static_cast<int>(axes.size())) {
    RCLCPP_ERROR_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Invalid axis index " << axis_index << ". Joystick has " << axes.size() << " axes.");
    return 0.0f;
  }
  return axes[axis_index];
}

void URTeleopNode::joy_callback(const sensor_msgs::msg::Joy& joy) {
  using Request = std_srvs::srv::Trigger::Request;

  constexpr std::string_view kFreedriveController = "freedrive_mode_controller";
  constexpr std::string_view kTrajectoryController = "scaled_joint_trajectory_controller";
  constexpr std::string_view kVelocityController = "forward_velocity_controller";

  if (previous_buttons_.size() != joy.buttons.size()) {
    previous_buttons_ = std::vector<int32_t>(joy.buttons.size());
  }

  const auto is_rising_edge = [this, &joy](const int button_index) {
    return this->is_button_pressed(joy.buttons, button_index) &&
           !this->is_button_pressed(previous_buttons_, button_index);
  };

  // Gripper release gets priority over gripper grip in the event that the
  // buttons are pressed simultaneously.
  if (is_rising_edge(gripper_release_button_)) {
    std::ignore = gripper_release_client_->async_send_request(std::make_shared<Request>());
  } else if (is_rising_edge(gripper_grip_button_)) {
    std::ignore = gripper_grip_client_->async_send_request(std::make_shared<Request>());
  }

  if (is_rising_edge(zero_ftsensor_button_)) {
    std::ignore = zero_ftsensor_client_->async_send_request(std::make_shared<Request>());
  }

  if (is_rising_edge(reset_pose_button_)) {
    const auto result = this->reset_pose();
    if (result) {
      RCLCPP_INFO(this->get_logger(), "Pose reset completed.");
    } else {
      RCLCPP_WARN(this->get_logger(), "%s", result.error().c_str());
    }
  }

  switch (control_mode_) {
    case URTeleopControlMode::kNotControlling:
      if (this->is_button_pressed(joy.buttons, freedrive_deadman_button_)) {
        this->switch_controllers(kFreedriveController, kTrajectoryController);
        control_mode_ = URTeleopControlMode::kFreedrive;
      } else if (this->is_button_pressed(joy.buttons, servo_deadman_button_)) {
        servo_state_.prev_commanded_ee_twist.setZero();
        servo_state_.target_ee_twist.setZero();
        this->switch_controllers(kVelocityController, kTrajectoryController);
        control_mode_ = URTeleopControlMode::kServo;
      } else if (this->is_button_pressed(joy.buttons, wholebody_deadman_button_)) {
        servo_state_.prev_commanded_ee_twist.setZero();
        servo_state_.target_ee_twist.setZero();
        this->switch_controllers(kVelocityController, kTrajectoryController);
        control_mode_ = URTeleopControlMode::kWholeBody;
      }
      break;

    case URTeleopControlMode::kFreedrive:
      if (!this->is_button_pressed(joy.buttons, freedrive_deadman_button_)) {
        this->switch_controllers(kTrajectoryController, kFreedriveController);
        control_mode_ = URTeleopControlMode::kNotControlling;
      }
      break;

    case URTeleopControlMode::kServo:
      if (!this->is_button_pressed(joy.buttons, servo_deadman_button_)) {
        this->switch_controllers(kTrajectoryController, kVelocityController);
        control_mode_ = URTeleopControlMode::kNotControlling;
      }
      break;

    case URTeleopControlMode::kWholeBody:
      if (!this->is_button_pressed(joy.buttons, wholebody_deadman_button_)) {
        this->switch_controllers(kTrajectoryController, kVelocityController);
        control_mode_ = URTeleopControlMode::kNotControlling;

        std_msgs::msg::Bool message;
        message.data = false;
        wholebody_keepalive_pub_->publish(message);
      }
      break;
  }

  if (control_mode_ == URTeleopControlMode::kServo ||
      control_mode_ == URTeleopControlMode::kWholeBody) {
    servo_state_.target_ee_twist[0] = this->get_axis_value(joy.axes, servo_x_axis_);
    servo_state_.target_ee_twist[1] = this->get_axis_value(joy.axes, servo_y_axis_);
    servo_state_.target_ee_twist[2] = this->get_axis_value(joy.axes, servo_z_axis_);
    servo_state_.target_ee_twist.head(3) *= servo_linear_scale_;

    servo_state_.target_ee_twist[3] = this->get_axis_value(joy.axes, servo_roll_axis_);
    servo_state_.target_ee_twist[4] = this->get_axis_value(joy.axes, servo_pitch_axis_);
    servo_state_.target_ee_twist[5] = this->get_axis_value(joy.axes, servo_yaw_axis_);
    servo_state_.target_ee_twist.tail(3) *= servo_angular_scale_;
  }

  previous_buttons_ = joy.buttons;
}
