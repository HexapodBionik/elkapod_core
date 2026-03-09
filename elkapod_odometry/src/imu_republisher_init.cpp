#include <cstdio>

#include "../include/elkapod_odometry/imu_republisher.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto executor = rclcpp::executors::SingleThreadedExecutor(rclcpp::ExecutorOptions());
  auto my_node = std::make_shared<imu_republisher::ImuRepublisher>();

  executor.add_node(my_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}