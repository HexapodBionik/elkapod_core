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
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <string>

using namespace std::chrono_literals;

class ElkapodTouchSensorRelay : public rclcpp::Node {
 public:
  ElkapodTouchSensorRelay();

 private:
  void fsrCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr contacts);
  void collisionPubCallback();
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr fsr_sub_1_, fsr_sub_2_,
      fsr_sub_3_, fsr_sub_4_, fsr_sub_5_, fsr_sub_6_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr my_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 6> force_cache_;
  std::array<double, 6> last_contact_;
  static inline constexpr double treshold_ = 0.08;
};

#endif  // ELKAPOD_TOUCH_SENSOR_RELAY_HPP
