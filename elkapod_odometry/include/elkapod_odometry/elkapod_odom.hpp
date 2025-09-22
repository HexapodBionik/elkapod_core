//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef ELKAPOD_ODOM_HPP
#define ELKAPOD_ODOM_HPP

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <string>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "elkapod_leg_kinematics.hpp"

using namespace std::chrono_literals;

class ElkapodOdom : public rclcpp::Node {
 public:
  ElkapodOdom();

 private:
  void collisionSubCallback(const std_msgs::msg::Int8MultiArray::SharedPtr contacts);
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr joint_states);
  void odomCallback();
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr collision_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 6> contact_cache_;
  std::array<double, 18> leg_angles_;
  std::array<double, 6> base_link_rotations_;
  std::vector<Eigen::Vector3d> base_link_translations_;

  bool joint_states_initialized_ = false;
  bool position_initialized_ = false;
  Eigen::Matrix4d odom_pose_;
  std::vector<Eigen::Vector3d> last_leg_positions_;
  std::unique_ptr<KinematicsSolver> solver_;

};

#endif  // ELKAPOD_ODOM_HPP
