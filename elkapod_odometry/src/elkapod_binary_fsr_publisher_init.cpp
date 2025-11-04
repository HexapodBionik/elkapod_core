#include <cstdio>

#include "../include/elkapod_odometry/elkapod_binary_fsr_publisher.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto executor = rclcpp::executors::SingleThreadedExecutor();
  auto my_node = std::make_shared<ElkapodBinaryFsrPublisher>();

  executor.add_node(my_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}