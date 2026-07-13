#include "ur_teleop_node.hpp"

#include <functional>

using std::placeholders::_1;

std::shared_ptr<URTeleopNode> URTeleopNode::create() {
  auto node = std::shared_ptr<URTeleopNode>(new URTeleopNode());
  node->setup_moveit_servo();
  return node;
}

URTeleopNode::URTeleopNode() : Node("ur_teleop_node") {
  update_frequency_ = this->declare_parameter<double>("update_frequency", 500.0);
  gripper_grip_button_ = this->declare_parameter<int>("gripper_grip_button", 1);
  gripper_release_button_ = this->declare_parameter<int>("gripper_release_button", 2);
  freedrive_deadman_button_ = this->declare_parameter<int>("freedrive_deadman_button", 6);
  servo_deadman_button_ = this->declare_parameter<int>("servo_deadman_button", 4);
  wholebody_deadman_button_ = this->declare_parameter<int>("wholebody_deadman_button", 7);
  reset_pose_button_ = this->declare_parameter<int>("reset_pose_button", 10);
  zero_ftsensor_button_ = this->declare_parameter<int>("zero_ftsensor_button", 3);
  servo_x_axis_ = this->declare_parameter<int>("servo_x_axis", 2);
  servo_y_axis_ = this->declare_parameter<int>("servo_y_axis", 3);
  servo_z_axis_ = this->declare_parameter<int>("servo_z_axis", 1);
  servo_roll_axis_ = this->declare_parameter<int>("servo_roll_axis", 5);
  servo_pitch_axis_ = this->declare_parameter<int>("servo_pitch_axis", 4);
  servo_yaw_axis_ = this->declare_parameter<int>("servo_yaw_axis", 0);
  servo_angular_scale_ = this->declare_parameter<double>("servo_angular_scale", 0.05);
  servo_linear_scale_ = this->declare_parameter<double>("servo_linear_scale", 0.05);
  servo_max_linear_accel_ = this->declare_parameter<double>("servo_max_linear_accel", 0.05);
  servo_max_angular_accel_ = this->declare_parameter<double>("servo_max_angular_accel", 0.05);

  reset_pose_params_.move_group_name =
      this->declare_parameter<std::string>("reset_pose.move_group_name", "ur_manipulator");
  reset_pose_params_.planning_time_sec =
      this->declare_parameter<double>("reset_pose.planning_time_sec", 5.0);
  reset_pose_params_.max_velocity_scaling =
      this->declare_parameter<double>("reset_pose.max_velocity_scaling", 0.2);
  reset_pose_params_.max_acceleration_scaling =
      this->declare_parameter<double>("reset_pose.max_acceleration_scaling", 0.2);

  const std::string base_mode_str =
      this->declare_parameter<std::string>("base_movement_mode", "none");
  if (base_mode_str == "full") {
    base_mode_ = ServoBaseMovementMode::kFull;
  }

  gripper_grip_client_ = this->create_client<std_srvs::srv::Trigger>("/vg10/grip");
  gripper_release_client_ = this->create_client<std_srvs::srv::Trigger>("/vg10/release");
  switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
  zero_ftsensor_client_ =
      this->create_client<std_srvs::srv::Trigger>("/io_and_status_controller/zero_ftsensor");

  freedrive_keepalive_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/freedrive_mode_controller/enable_freedrive_mode", 1);
  wholebody_keepalive_pub_ = this->create_publisher<std_msgs::msg::Bool>("/qp/enable", 1);
  // servo_control_pub_ is initialized in setup_moveit_servo since the topic is from servo params.
  base_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  wholebody_desired_twist_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("/qp/cmd_twist_teleop", 1);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind(&URTeleopNode::joy_callback, this, _1));
}
