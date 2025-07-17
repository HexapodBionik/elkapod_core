#include <cstdio>

#include "../include/elkapod_kinematics/elkapod_leg_controller.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto my_node = std::make_shared<ElkapodLegPublisher>();
  my_node->init();

  rclcpp::spin(my_node);
  rclcpp::shutdown();
  return 0;
}
