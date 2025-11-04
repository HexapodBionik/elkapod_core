//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef ELKAPOD_TOUCH_SENSOR_RELAY_HPP
#define ELKAPOD_TOUCH_SENSOR_RELAY_HPP

#include <array>
#include <chrono>
#include <elkapod_msgs/msg/float64_array_stamped.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <string>

using namespace std::chrono_literals;

class ElkapodBinaryFsrPublisher : public rclcpp::Node {
 public:
  ElkapodBinaryFsrPublisher();

 private:
  void fsrCallback(elkapod_msgs::msg::Float64ArrayStamped fsr_values);
  void collisionPubCallback();
  rclcpp::Subscription<elkapod_msgs::msg::Float64ArrayStamped>::SharedPtr fsr_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr my_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 6> last_contact_;
  double threshold_;
  double reset_delay_;
};

#endif  // ELKAPOD_TOUCH_SENSOR_RELAY_HPP
