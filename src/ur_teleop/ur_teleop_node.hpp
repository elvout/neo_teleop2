#pragma once

#include <atomic>
#include <cstdint>
#include <expected>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace moveit::planning_interface {
class MoveGroupInterface;
}

namespace moveit_servo {
class Servo;
}

namespace planning_scene_monitor {
class PlanningSceneMonitor;
}

using Vector6d = Eigen::Vector<double, 6>;

enum class URTeleopControlMode {
  // This node is not controlling the UR.
  kNotControlling,
  // This node is controlling the UR via freedrive mode.
  kFreedrive,
  // This node is controlling the UR at its end-effector via MoveIt Servo.
  kServo,
  // This node is sending twists to a whole-body controller.
  kWholeBody,
};

// Optionally move the base while in kServo mode.
enum class ServoBaseMovementMode {
  // This node does not send any base commands.
  kNone,
  // This node uses base commands to fully satisfy v_x, v_y, omega_z.
  kFull,
};

struct ServoControlState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vector6d target_ee_twist = Vector6d::Zero(6);
  Vector6d prev_commanded_ee_twist = Vector6d::Zero(6);
};

struct ResetPoseParams {
  std::string move_group_name;
  double planning_time_sec;
  double max_velocity_scaling;
  double max_acceleration_scaling;
};

class URTeleopNode : public rclcpp::Node {
 public:
  static std::shared_ptr<URTeleopNode> create();

  void send_cmd();

 private:
  URTeleopNode();

  // MoveIt Servo setup requires shared_from_this(), which is not valid within
  // the constructor.
  void setup_moveit_servo();

  void switch_controllers(std::string_view activate, std::string_view deactivate);

  std::expected<void, std::string> reset_pose();

  bool is_button_pressed(std::span<const int32_t> buttons, int button_index) const;
  float get_axis_value(std::span<const float> axes, int axis_index) const;
  void joy_callback(const sensor_msgs::msg::Joy& joy);

  // Parameters
 public:
  double update_frequency_ = 0.0;

 private:
  int gripper_grip_button_ = -1;
  int gripper_release_button_ = -1;
  int freedrive_deadman_button_ = -1;
  int wholebody_deadman_button_ = -1;
  int reset_pose_button_ = -1;
  int zero_ftsensor_button_ = -1;

  int servo_deadman_button_ = -1;
  int servo_x_axis_ = -1;
  int servo_y_axis_ = -1;
  int servo_z_axis_ = -1;
  int servo_roll_axis_ = -1;
  int servo_pitch_axis_ = -1;
  int servo_yaw_axis_ = -1;
  double servo_angular_scale_ = 0.0;
  double servo_linear_scale_ = 0.0;
  double servo_max_linear_accel_ = 0.0;
  double servo_max_angular_accel_ = 0.0;

  ResetPoseParams reset_pose_params_;

  // Internal state
  std::vector<int32_t> previous_buttons_;
  URTeleopControlMode control_mode_ = URTeleopControlMode::kNotControlling;
  ServoControlState servo_state_;
  ServoBaseMovementMode base_mode_ = ServoBaseMovementMode::kNone;

  // ROS 2 machinery
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_grip_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_release_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_controller_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr zero_ftsensor_client_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr freedrive_keepalive_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wholebody_keepalive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_control_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr wholebody_desired_twist_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::shared_ptr<moveit_servo::Servo> servo_;
  std::atomic<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>
      move_group_interface_;
  std::jthread move_group_setup_thread_;
};
