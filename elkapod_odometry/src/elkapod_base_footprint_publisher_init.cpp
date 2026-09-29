#include <cstdio>

#include "../include/elkapod_odometry/elkapod_base_footprint_publisher.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto executor = rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 4);
  auto my_node = std::make_shared<ElkapodBaseFootprintPublisher>();

  executor.add_node(my_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}