#include <rclcpp/rclcpp.hpp>

#include "rec_rep2_servo/compliant_torque_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rec_rep2_servo::ComplianceTorqueNode>());
  rclcpp::shutdown();
  return 0;
}
