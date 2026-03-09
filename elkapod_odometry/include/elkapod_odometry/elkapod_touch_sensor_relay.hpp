//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef ELKAPOD_TOUCH_SENSOR_RELAY_HPP
#define ELKAPOD_TOUCH_SENSOR_RELAY_HPP

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

#include "elkapod_msgs/msg/float64_array_stamped.hpp"

using namespace std::chrono_literals;

class ElkapodTouchSensorRelay : public rclcpp::Node {
 public:
  ElkapodTouchSensorRelay();

 private:
  void fsrCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr contacts, int leg_id);
  void timerCallback();
  rclcpp::Publisher<elkapod_msgs::msg::Float64ArrayStamped>::SharedPtr my_publisher_;

  std::mutex data_mutex_;
  std::vector<double> fsr_buffer_;
  std::array<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr, 6> fsr_subs_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

#endif  // ELKAPOD_TOUCH_SENSOR_RELAY_HPP
