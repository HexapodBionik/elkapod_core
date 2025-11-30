//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef IMU_REPUBLIHSER_HPP
#define IMU_REPUBLIHSER_HPP

#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace imu_republisher {
class ImuRepublisher : public rclcpp::Node {
 public:
  ImuRepublisher();

 private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::default_random_engine generator_;
  std::vector<std::normal_distribution<double>> noise_dist_;
};
};  // namespace imu_republisher

#endif  // IMU_REPUBLIHSER_HPP
