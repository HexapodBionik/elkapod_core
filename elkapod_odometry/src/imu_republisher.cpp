#include "../include/elkapod_odometry/imu_republisher.hpp"

ImuRepublisher::ImuRepublisher() : Node("imu_republisher") {
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_raw", 10, std::bind(&ImuRepublisher::imuCallback, this, std::placeholders::_1));

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

  RCLCPP_INFO(get_logger(), "IMU republisher started!");
}

void ImuRepublisher::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  msg->orientation_covariance = {0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1};

  msg->header.frame_id = "imu_link";
  imu_pub_->publish(*msg);
}
