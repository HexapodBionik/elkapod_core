#include <cstdio>
#include "../include/elkapod_motion_manager/elkapod_motion_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto my_node = std::make_shared<ElkapodMotionManager>();
  my_node->initNode();

  rclcpp::spin(my_node);
  rclcpp::shutdown();
  return 0;
}
