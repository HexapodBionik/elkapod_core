#include <cstdio>

#include "../include/elkapod_gait_gen_cpp/elkapod_gait_gen.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto my_node = std::make_shared<ElkapodGaitGen>();

  rclcpp::spin(my_node);
  rclcpp::shutdown();
  return 0;
}
