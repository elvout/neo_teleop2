#include <moveit/move_group_interface/move_group_interface.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_servo/servo.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <tuple>

using std::placeholders::_1;
using Vector6d = Eigen::Vector<double, 6>;

enum class URTeleopControlMode {
  // This node is not controlling the UR.
  kNotControlling,
  // This node is controlling the UR via freedrive mode.
  kFreedrive,
  // This node is controlling the UR end-effector via servo.
  kServo,
  // This node is sending twists to a whole-body controller.
  kWholeBody,
};

// Optionally move the base while in kServo mode.
enum class BaseMovementMode {
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

namespace {
Vector6d TransformTwist(const Vector6d& twist, const Eigen::Isometry3d& T_new_old) {
  Vector6d new_twist = Vector6d::Zero();

  new_twist.segment(0, 3) = T_new_old.rotation() * twist.segment(0, 3);
  new_twist.segment(3, 3) = T_new_old.rotation() * twist.segment(3, 3);

  return new_twist;
}
}  // namespace

class URTeleop : public rclcpp::Node {
 public:
  URTeleop();

  // moveit_servo setup requires calls to this->shared_from_this, which are not
  // available in the constructor.
  void setup_moveit_servo();

  void send_cmd();

 private:
  bool is_valid_button(const int button, const sensor_msgs::msg::Joy::SharedPtr joy) const;
  bool is_valid_axis(const int axis, const sensor_msgs::msg::Joy::SharedPtr joy) const;

  bool is_button_pressed(const int button, const sensor_msgs::msg::Joy::SharedPtr joy) const;
  float get_axis_value(const int axis, const sensor_msgs::msg::Joy::SharedPtr joy) const;

  void switch_controllers(const std::string& activate, const std::string& deactivate);

  bool pose_reset(std::string& status_message);

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy);

  // Parameters
 public:
  double update_frequency_ = 0.0;

 private:
  int gripper_grip_button_ = -1;
  int gripper_release_button_ = -1;
  int freedrive_deadman_button_ = -1;
  int wholebody_deadman_button_ = -1;
  int pose_reset_button_ = -1;
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

  std::string pose_reset_move_group_name_;
  std::string pose_reset_named_target_;
  double pose_reset_planning_time_sec_ = 5.0;
  double pose_reset_max_velocity_scaling_ = 0.2;
  double pose_reset_max_acceleration_scaling_ = 0.2;

  // Internal state
  URTeleopControlMode control_mode_ = URTeleopControlMode::kNotControlling;
  ServoControlState servo_state_;
  BaseMovementMode base_mode_ = BaseMovementMode::kNone;
  bool pose_reset_button_was_pressed_ = false;

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
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

URTeleop::URTeleop() : Node("ur_teleop_node") {
  update_frequency_ = this->declare_parameter<double>("update_frequency", 500.0);
  gripper_grip_button_ = this->declare_parameter<int>("gripper_grip_button", 1);
  gripper_release_button_ = this->declare_parameter<int>("gripper_release_button", 2);
  freedrive_deadman_button_ = this->declare_parameter<int>("freedrive_deadman_button", 6);
  servo_deadman_button_ = this->declare_parameter<int>("servo_deadman_button", 4);
  wholebody_deadman_button_ = this->declare_parameter<int>("wholebody_deadman_button", 7);
  pose_reset_button_ = this->declare_parameter<int>("pose_reset_button", 10);
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

  pose_reset_move_group_name_ =
      this->declare_parameter<std::string>("pose_reset_move_group_name", "ur_manipulator");
  pose_reset_named_target_ =
      this->declare_parameter<std::string>("pose_reset_named_target", "home");
  pose_reset_planning_time_sec_ =
      this->declare_parameter<double>("pose_reset_planning_time_sec", 5.0);
  pose_reset_max_velocity_scaling_ =
      this->declare_parameter<double>("pose_reset_max_velocity_scaling", 0.2);
  pose_reset_max_acceleration_scaling_ =
      this->declare_parameter<double>("pose_reset_max_acceleration_scaling", 0.2);

  const std::string base_mode_str =
      this->declare_parameter<std::string>("base_movement_mode", "none");
  if (base_mode_str == "full") {
    base_mode_ = BaseMovementMode::kFull;
  }

  // TODO(elvout): gripper service names could be params
  // TODO(elvout): wait for services
  // https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html#write-the-client-node
  gripper_grip_client_ = this->create_client<std_srvs::srv::Trigger>("/vg10/grip");
  gripper_release_client_ = this->create_client<std_srvs::srv::Trigger>("/vg10/release");
  switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
  zero_ftsensor_client_ =
      this->create_client<std_srvs::srv::Trigger>("/io_and_status_controller/zero_ftsensor");

  freedrive_keepalive_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/freedrive_mode_controller/enable_freedrive_mode", 1);
  wholebody_keepalive_pub_ = this->create_publisher<std_msgs::msg::Bool>("/qp/enable", 1);
  base_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  wholebody_desired_twist_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("/qp/cmd_twist_teleop", 1);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind(&URTeleop::joy_callback, this, _1));
}

void URTeleop::setup_moveit_servo() {
  const std::string servo_param_namespace = "moveit_servo";
  auto servo_param_listener =
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

  try {
    move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        this->shared_from_this(), pose_reset_move_group_name_, std::shared_ptr<tf2_ros::Buffer>(),
        rclcpp::Duration::from_seconds(0.25));
    if (!move_group_interface_->getMoveGroupClient().wait_for_action_server(
            std::chrono::duration<double>(0.25))) {
      throw std::runtime_error("Timed out waiting for move_group action server");
    }
  } catch (const std::exception& ex) {
    move_group_interface_.reset();
    RCLCPP_ERROR(
        this->get_logger(),
        "MoveGroupInterface initialization failed for group '%s': %s. Is move_group running?",
        pose_reset_move_group_name_.c_str(), ex.what());
  }

  if (move_group_interface_) {
    move_group_interface_->setPlanningTime(pose_reset_planning_time_sec_);
    move_group_interface_->setMaxVelocityScalingFactor(
        std::clamp(pose_reset_max_velocity_scaling_, 0.0, 1.0));
    move_group_interface_->setMaxAccelerationScalingFactor(
        std::clamp(pose_reset_max_acceleration_scaling_, 0.0, 1.0));
  }
}

bool URTeleop::is_valid_button(const int button, const sensor_msgs::msg::Joy::SharedPtr joy) const {
  return 0 <= button && button < static_cast<int>(joy->buttons.size());
}

bool URTeleop::is_valid_axis(const int axis, const sensor_msgs::msg::Joy::SharedPtr joy) const {
  return 0 <= axis && axis < static_cast<int>(joy->axes.size());
}

bool URTeleop::is_button_pressed(const int button,
                                 const sensor_msgs::msg::Joy::SharedPtr joy) const {
  if (!is_valid_button(button, joy)) {
    RCLCPP_ERROR_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Invalid button " << button << ". Joystick has " << joy->buttons.size() << " buttons.");
    return false;
  }
  return static_cast<bool>(joy->buttons[button]);
}

float URTeleop::get_axis_value(const int axis, const sensor_msgs::msg::Joy::SharedPtr joy) const {
  if (!is_valid_axis(axis, joy)) {
    RCLCPP_ERROR_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Invalid axis " << axis << ". Joystick has " << joy->axes.size() << " axes.");
    return 0.0f;
  }
  return joy->axes[axis];
}

void URTeleop::switch_controllers(const std::string& activate, const std::string& deactivate) {
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = {activate};
  request->deactivate_controllers = {deactivate};
  request->activate_asap = true;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  request->timeout.sec = 1;
  std::ignore = switch_controller_client_->async_send_request(request);
}

bool URTeleop::pose_reset(std::string& status_message) {
  if (!move_group_interface_) {
    status_message =
        "Pose reset unavailable: MoveGroupInterface is not initialized. Is move_group running?";
    return false;
  }

  if (control_mode_ != URTeleopControlMode::kNotControlling) {
    status_message = "Pose reset rejected: teleop is active. Release deadman buttons first.";
    return false;
  }

  move_group_interface_->setStartStateToCurrentState();
  if (!move_group_interface_->setNamedTarget(pose_reset_named_target_)) {
    status_message = "Named target '" + pose_reset_named_target_ +
                     "' does not exist for move group '" + pose_reset_move_group_name_ + "'.";
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(move_group_interface_->plan(plan))) {
    status_message = "Planning failed for named target '" + pose_reset_named_target_ + "'.";
    return false;
  }
  if (!static_cast<bool>(move_group_interface_->execute(plan))) {
    status_message = "Execution failed for named target '" + pose_reset_named_target_ + "'.";
    return false;
  }

  status_message = "Arm pose reset to '" + pose_reset_named_target_ + "'.";
  return true;
}

void URTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {
  // Gripper release gets priority over gripper grip in the event that the
  // buttons are pressed simultaneously.
  if (this->is_button_pressed(gripper_release_button_, joy)) {
    const auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::ignore = gripper_release_client_->async_send_request(request);
  } else if (this->is_button_pressed(gripper_grip_button_, joy)) {
    const auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::ignore = gripper_grip_client_->async_send_request(request);
  }

  if (this->is_button_pressed(zero_ftsensor_button_, joy)) {
    const auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::ignore = zero_ftsensor_client_->async_send_request(request);
  }

  if (pose_reset_button_ >= 0) {
    const bool pose_reset_button_pressed = this->is_button_pressed(pose_reset_button_, joy);
    if (pose_reset_button_pressed && !pose_reset_button_was_pressed_) {
      std::string pose_reset_status;
      const bool pose_reset_success = this->pose_reset(pose_reset_status);
      if (pose_reset_success) {
        RCLCPP_INFO(this->get_logger(), "%s", pose_reset_status.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "%s", pose_reset_status.c_str());
      }
    }
    pose_reset_button_was_pressed_ = pose_reset_button_pressed;
  }

  if (control_mode_ == URTeleopControlMode::kNotControlling &&
      this->is_button_pressed(freedrive_deadman_button_, joy)) {
    this->switch_controllers("freedrive_mode_controller", "scaled_joint_trajectory_controller");
    control_mode_ = URTeleopControlMode::kFreedrive;
  } else if (control_mode_ == URTeleopControlMode::kFreedrive &&
             !this->is_button_pressed(freedrive_deadman_button_, joy)) {
    this->switch_controllers("scaled_joint_trajectory_controller", "freedrive_mode_controller");
    control_mode_ = URTeleopControlMode::kNotControlling;
  } else if (control_mode_ == URTeleopControlMode::kNotControlling &&
             this->is_button_pressed(servo_deadman_button_, joy)) {
    servo_state_.prev_commanded_ee_twist.setZero();
    servo_state_.target_ee_twist.setZero();
    this->switch_controllers("forward_velocity_controller", "scaled_joint_trajectory_controller");
    control_mode_ = URTeleopControlMode::kServo;
  } else if (control_mode_ == URTeleopControlMode::kServo &&
             !this->is_button_pressed(servo_deadman_button_, joy)) {
    this->switch_controllers("scaled_joint_trajectory_controller", "forward_velocity_controller");
    control_mode_ = URTeleopControlMode::kNotControlling;
  } else if (control_mode_ == URTeleopControlMode::kNotControlling &&
             this->is_button_pressed(wholebody_deadman_button_, joy)) {
    servo_state_.prev_commanded_ee_twist.setZero();
    servo_state_.target_ee_twist.setZero();
    this->switch_controllers("forward_velocity_controller", "scaled_joint_trajectory_controller");
    control_mode_ = URTeleopControlMode::kWholeBody;
  } else if (control_mode_ == URTeleopControlMode::kWholeBody &&
             !this->is_button_pressed(wholebody_deadman_button_, joy)) {
    this->switch_controllers("scaled_joint_trajectory_controller", "forward_velocity_controller");
    control_mode_ = URTeleopControlMode::kNotControlling;

    auto bool_msg = std_msgs::msg::Bool();
    bool_msg.data = false;
    wholebody_keepalive_pub_->publish(bool_msg);
  }

  if (control_mode_ == URTeleopControlMode::kServo ||
      control_mode_ == URTeleopControlMode::kWholeBody) {
    servo_state_.target_ee_twist[0] = this->get_axis_value(servo_x_axis_, joy);
    servo_state_.target_ee_twist[1] = this->get_axis_value(servo_y_axis_, joy);
    servo_state_.target_ee_twist[2] = this->get_axis_value(servo_z_axis_, joy);
    servo_state_.target_ee_twist.head(3) *= servo_linear_scale_;

    servo_state_.target_ee_twist[3] = this->get_axis_value(servo_roll_axis_, joy);
    servo_state_.target_ee_twist[4] = this->get_axis_value(servo_pitch_axis_, joy);
    servo_state_.target_ee_twist[5] = this->get_axis_value(servo_yaw_axis_, joy);
    servo_state_.target_ee_twist.tail(3) *= servo_angular_scale_;
  }
}

void URTeleop::send_cmd() {
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

    if (base_mode_ == BaseMovementMode::kFull) {
      // split into components: base takes vx, vy, omegaz
      Vector6d next_command_base_twist = Vector6d::Zero();
      next_command_base_twist[5] = next_command_ee_twist[5];

      const double phi =
          std::atan2(T_baselink_ee.translation().y(), T_baselink_ee.translation().x());
      const double disp =
          Eigen::Vector2d{T_baselink_ee.translation().x(), T_baselink_ee.translation().y()}.norm();
      next_command_base_twist.segment(0, 2) =
          next_command_ee_twist.segment(0, 2) -
          Eigen::Rotation2Dd(phi + M_PI / 2.0) *
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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto urteleop_node = std::make_shared<URTeleop>();
  urteleop_node->setup_moveit_servo();

  rclcpp::Rate loop_rate(urteleop_node->update_frequency_);
  while (rclcpp::ok()) {
    rclcpp::spin_some(urteleop_node);
    urteleop_node->send_cmd();
    loop_rate.sleep();
  }

  return 0;
}
