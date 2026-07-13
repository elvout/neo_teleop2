#include <rclcpp/rclcpp.hpp>

#include "ur_teleop_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto ur_teleop_node = URTeleopNode::create();

  rclcpp::Rate loop_rate(ur_teleop_node->update_frequency_);
  while (rclcpp::ok()) {
    rclcpp::spin_some(ur_teleop_node);
    ur_teleop_node->send_cmd();
    loop_rate.sleep();
  }

  return 0;
}
