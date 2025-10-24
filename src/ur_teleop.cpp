#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <moveit_servo/servo.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using std::placeholders::_1;

enum class URTeleopControlMode {
  // This node is not controlling the UR.
  kNotControlling,
  // This node is controlling the UR via freedrive mode.
  kFreedrive,
  // This node is controlling the UR end-effector via servo.
  kServo,
};

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

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy);

 private:
  // Parameters
  int gripper_grip_button_ = -1;
  int gripper_release_button_ = -1;
  int freedrive_deadman_button_ = -1;

  int servo_deadman_button_ = -1;
  int servo_x_axis_ = -1;
  int servo_y_axis_ = -1;
  int servo_z_axis_ = -1;
  int servo_roll_axis_ = -1;
  int servo_pitch_axis_ = -1;
  int servo_yaw_axis_ = -1;
  double servo_angular_scale_ = 0.0;
  double servo_linear_scale_ = 0.0;
  double servo_twist_smooth_factor_ = 1.0;

  // Internal state
  URTeleopControlMode control_mode_ = URTeleopControlMode::kNotControlling;
  // linear.x, linear.y, linear.z, angular.x, angular.y, angular.z
  Eigen::VectorXd target_ee_twist_ = Eigen::VectorXd::Zero(6);

  // ROS 2 machinery
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_grip_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_release_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_controller_client_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr freedrive_keepalive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_control_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::shared_ptr<moveit_servo::Servo> servo_;
};

URTeleop::URTeleop() : Node("ur_teleop") {
  gripper_grip_button_ = this->declare_parameter<int>("gripper_grip_button", 1);
  gripper_release_button_ = this->declare_parameter<int>("gripper_release_button", 2);
  freedrive_deadman_button_ = this->declare_parameter<int>("freedrive_deadman_button", 6);
  servo_deadman_button_ = this->declare_parameter<int>("servo_deadman_button", 4);
  servo_x_axis_ = this->declare_parameter<int>("servo_x_axis", 2);
  servo_y_axis_ = this->declare_parameter<int>("servo_y_axis", 3);
  servo_z_axis_ = this->declare_parameter<int>("servo_z_axis", 1);
  servo_roll_axis_ = this->declare_parameter<int>("servo_roll_axis", 5);
  servo_pitch_axis_ = this->declare_parameter<int>("servo_pitch_axis", 4);
  servo_yaw_axis_ = this->declare_parameter<int>("servo_yaw_axis", 0);
  servo_angular_scale_ = this->declare_parameter<double>("servo_angular_scale", 0.05);
  servo_linear_scale_ = this->declare_parameter<double>("servo_linear_scale", 0.05);
  servo_twist_smooth_factor_ = this->declare_parameter<double>("servo_twist_smooth_factor", 1.0);

  // TODO(elvout): gripper service names could be params
  // TODO(elvout): wait for services
  // https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html#write-the-client-node
  gripper_grip_client_ = this->create_client<std_srvs::srv::Trigger>("/vg10/grip");
  gripper_release_client_ = this->create_client<std_srvs::srv::Trigger>("/vg10/release");
  switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

  freedrive_keepalive_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/freedrive_mode_controller/enable_freedrive_mode", 1);

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
    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid button " << button << ". Joystick has "
                                                              << joy->buttons.size()
                                                              << " buttons.");
    return false;
  }
  return static_cast<bool>(joy->buttons[button]);
}

float URTeleop::get_axis_value(const int axis, const sensor_msgs::msg::Joy::SharedPtr joy) const {
  if (!is_valid_axis(axis, joy)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid axis " << axis << ". Joystick has "
                                                            << joy->axes.size() << " axes.");
    return false;
  }
  return joy->axes[axis];
}

void URTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {
  // Gripper release gets priority over gripper grip in the event that the
  // buttons are pressed simultaneously.
  if (this->is_button_pressed(gripper_release_button_, joy)) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = gripper_release_client_->async_send_request(request);
  } else if (this->is_button_pressed(gripper_grip_button_, joy)) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = gripper_grip_client_->async_send_request(request);
  }

  if (control_mode_ == URTeleopControlMode::kNotControlling &&
      this->is_button_pressed(freedrive_deadman_button_, joy)) {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = {"freedrive_mode_controller"};
    request->deactivate_controllers = {"scaled_joint_trajectory_controller"};
    request->activate_asap = true;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->timeout.sec = 1;

    auto result = switch_controller_client_->async_send_request(request);
    control_mode_ = URTeleopControlMode::kFreedrive;
  } else if (control_mode_ == URTeleopControlMode::kFreedrive &&
             !this->is_button_pressed(freedrive_deadman_button_, joy)) {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = {"scaled_joint_trajectory_controller"};
    request->deactivate_controllers = {"freedrive_mode_controller"};
    request->activate_asap = true;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->timeout.sec = 1;

    auto result = switch_controller_client_->async_send_request(request);
    control_mode_ = URTeleopControlMode::kNotControlling;
  } else if (control_mode_ == URTeleopControlMode::kNotControlling &&
             this->is_button_pressed(servo_deadman_button_, joy)) {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = {"forward_velocity_controller"};
    request->deactivate_controllers = {"scaled_joint_trajectory_controller"};
    request->activate_asap = true;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->timeout.sec = 1;

    auto result = switch_controller_client_->async_send_request(request);
    control_mode_ = URTeleopControlMode::kServo;
    target_ee_twist_.setZero();
  } else if (control_mode_ == URTeleopControlMode::kServo &&
             !this->is_button_pressed(servo_deadman_button_, joy)) {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = {"scaled_joint_trajectory_controller"};
    request->deactivate_controllers = {"forward_velocity_controller"};
    request->activate_asap = true;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->timeout.sec = 1;

    auto result = switch_controller_client_->async_send_request(request);
    control_mode_ = URTeleopControlMode::kNotControlling;
  }

  if (control_mode_ == URTeleopControlMode::kServo) {
    Eigen::VectorXd new_target_ee_twist = Eigen::VectorXd::Zero(6);
    new_target_ee_twist[0] = this->get_axis_value(servo_x_axis_, joy) * servo_linear_scale_;
    new_target_ee_twist[1] = this->get_axis_value(servo_y_axis_, joy) * servo_linear_scale_;
    new_target_ee_twist[2] = this->get_axis_value(servo_z_axis_, joy) * servo_linear_scale_;
    new_target_ee_twist[3] = this->get_axis_value(servo_roll_axis_, joy) * servo_angular_scale_;
    new_target_ee_twist[4] = this->get_axis_value(servo_pitch_axis_, joy) * servo_angular_scale_;
    new_target_ee_twist[5] = this->get_axis_value(servo_yaw_axis_, joy) * servo_angular_scale_;
    target_ee_twist_ = (1.0 - servo_twist_smooth_factor_) * target_ee_twist_ +
                       servo_twist_smooth_factor_ * new_target_ee_twist;
  }
}

void URTeleop::send_cmd() {
  if (control_mode_ == URTeleopControlMode::kFreedrive) {
    auto bool_msg = std_msgs::msg::Bool();
    bool_msg.data = true;
    freedrive_keepalive_pub_->publish(bool_msg);
  } else if (control_mode_ == URTeleopControlMode::kServo) {
    const moveit::core::RobotStatePtr robot_state =
        planning_scene_monitor_->getStateMonitor()->getCurrentState();

    const moveit_servo::TwistCommand command{"vg10_grasp_center", target_ee_twist_};
    // Note: Do not smooth the velocities by interpolating with a low (< 0.75)
    // coefficient. Doing so will cause the velocities to jitter the UR10e
    // dangerously due to the underlying (PD/PID?) controller. Smoothing the
    // twist works much better.
    const moveit_servo::KinematicState next_state = servo_->getNextJointState(robot_state, command);

    auto forward_vel_command =
        moveit_servo::composeMultiArrayMessage(servo_->getParams(), next_state);
    servo_control_pub_->publish(forward_vel_command);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto urteleop_node = std::make_shared<URTeleop>();
  urteleop_node->setup_moveit_servo();

  const double control_rate = 100;
  rclcpp::Rate loop_rate(control_rate);

  while (rclcpp::ok()) {
    rclcpp::spin_some(urteleop_node);
    urteleop_node->send_cmd();
    loop_rate.sleep();
  }

  return 0;
}
