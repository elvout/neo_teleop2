/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NeoTeleop : public rclcpp::Node
{
public:
  NeoTeleop()
  : Node("neo_teleop2_node")
  {
    // declare Parameters
    this->declare_parameter<double>("scale_linear_x", 0.4);
    this->declare_parameter<double>("scale_linear_y", 0.4);
    this->declare_parameter<double>("scale_angular_z", 0.6);
    this->declare_parameter<int>("axis_linear_x", 1);
    this->declare_parameter<int>("axis_linear_y", 0);
    this->declare_parameter<int>("axis_angular_z", 2);
    this->declare_parameter<double>("smooth_factor", 0.2);
    this->declare_parameter<int>("deadman_button", 5);
    this->declare_parameter<double>("joy_timeout", 1.);
    this->declare_parameter<int>("grip_button", 1);
    this->declare_parameter<int>("release_button", 2);

    // Get Paramters
    this->get_parameter("scale_linear_x", linear_scale_x);
    this->get_parameter("scale_linear_y", linear_scale_y);
    this->get_parameter("scale_angular_z", angular_scale_z);
    this->get_parameter("axis_linear_x", axis_linear_x);
    this->get_parameter("axis_linear_y", axis_linear_y);
    this->get_parameter("axis_angular_z", axis_angular_z);
    this->get_parameter("smooth_factor", smooth_factor);
    this->get_parameter("deadman_button", deadman_button);
    this->get_parameter("joy_timeout", joy_timeout);
    this->get_parameter("grip_button", grip_button);
    this->get_parameter("release_button", release_button);

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1,
      std::bind(&NeoTeleop::joy_callback, this, _1));

    // TODO(elvout): services should be params
    grip_client = this->create_client<std_srvs::srv::Trigger>("/vg10/grip");
    release_client = this->create_client<std_srvs::srv::Trigger>("/vg10/release");
    // TODO(elvout): wait for services
    // https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html
  }

  void send_cmd();

protected:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  geometry_msgs::msg::Twist cmd_vel;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr grip_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr release_client;

  double linear_scale_x = 0;
  double linear_scale_y = 0;
  double angular_scale_z = 0;
  double smooth_factor = 1;
  double joy_timeout = 0;
  int axis_linear_x = -1;
  int axis_linear_y = -1;
  int axis_angular_z = -1;
  int deadman_button = -1;
  int grip_button = -1;
  int release_button = -1;

  rclcpp::Time last_joy_time;
  double joy_command_x = 0;
  double joy_command_y = 0;
  double joy_command_z = 0;

  bool is_active = false;
  bool is_deadman_pressed = false;
};


void NeoTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  const int kNumAxesPresent = static_cast<int>(joy->axes.size());
  const int kNumButtonsPresent = static_cast<int>(joy->buttons.size());

  const auto is_valid_axis = [&](const int axis) -> bool {
    return axis >= 0 && axis < kNumAxesPresent;
  };
  const auto is_valid_button = [&](const int button) -> bool {
    return button >= 0 && button < kNumButtonsPresent;
  };

  this->is_deadman_pressed = is_valid_button(this->deadman_button) &&
                             static_cast<bool>(joy->buttons[this->deadman_button]);
  if (this->is_deadman_pressed) {
    this->is_active = true;
    this->last_joy_time = rclcpp::Clock().now();
  }

  if (is_valid_axis(this->axis_linear_x)) {
    joy_command_x = this->linear_scale_x * joy->axes[this->axis_linear_x];
  }
  if (is_valid_axis(this->axis_linear_y)) {
    joy_command_y = this->linear_scale_y * joy->axes[this->axis_linear_y];
  }
  if (is_valid_axis(this->axis_angular_z)) {
    joy_command_z = this->angular_scale_z * joy->axes[this->axis_angular_z];
  }

  // TODO(elvout): should check for nullptr / uninit sharedptr
  // TODO(elvout): is this appropriate? Should we wait for the futures to complete?
  if (is_valid_button(this->grip_button) && static_cast<bool>(joy->buttons[this->grip_button])) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = this->grip_client->async_send_request(request);
  } else if (is_valid_button(this->release_button) &&
             static_cast<bool>(joy->buttons[this->release_button])) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = release_client->async_send_request(request);
  }
}

void NeoTeleop::send_cmd()
{
  if (is_deadman_pressed) {
    // smooth inputs
    cmd_vel.linear.x = joy_command_x * smooth_factor + cmd_vel.linear.x * (1 - smooth_factor);
    cmd_vel.linear.y = joy_command_y * smooth_factor + cmd_vel.linear.y * (1 - smooth_factor);
    cmd_vel.angular.z = joy_command_z * smooth_factor + cmd_vel.angular.z * (1 - smooth_factor);

    // publish
    vel_pub->publish(cmd_vel);
  } else if (is_active) {
    if ((rclcpp::Clock().now() - last_joy_time).seconds() > joy_timeout) {
      cmd_vel = geometry_msgs::msg::Twist();      // set to all zero
      is_active = false;
    } else {
      // smooth towards zero
      cmd_vel.linear.x = cmd_vel.linear.x * (1 - smooth_factor);
      cmd_vel.linear.y = cmd_vel.linear.y * (1 - smooth_factor);
      cmd_vel.angular.z = cmd_vel.angular.z * (1 - smooth_factor);
    }
    // publish
    vel_pub->publish(cmd_vel);
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoTeleop>();
  double control_rate = 50;
  rclcpp::Rate loop_rate(control_rate);

  while (rclcpp::ok()) {
    nh->send_cmd();

    loop_rate.sleep();

    rclcpp::spin_some(nh);
  }

  return 0;
}
