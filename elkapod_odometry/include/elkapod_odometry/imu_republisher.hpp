//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef IMU_REPUBLIHSER_HPP
#define IMU_REPUBLIHSER_HPP

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

using namespace std::chrono_literals;

class ImuRepublisher : public rclcpp::Node {
 public:
  ImuRepublisher();

 private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

#endif  // IMU_REPUBLIHSER_HPP
