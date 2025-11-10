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
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <string>

#include "elkapod_core_lib/leg_kinematics.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
using KinematicsSolver = elkapod_core_lib::kinematics::KinematicsSolver;

class ElkapodOdom : public rclcpp::Node {
 public:
  ElkapodOdom();

 private:
  void collisionSubCallback(const std_msgs::msg::Int8MultiArray::SharedPtr contacts);
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr joint_states);
  void odomCallback();
  void tfCallback();
  nav_msgs::msg::Odometry fillOdomMsg(const Eigen::Matrix4d odom_pose,
                                      const Eigen::Matrix4d previous_odom_pose);

  Eigen::Vector4d findPlane(const Eigen::Matrix3Xd contact_points);
  Eigen::Vector3d findBaseFootprintCoords(Eigen::Vector4d plane);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr collision_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr broadcaster_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::array<double, 6> contact_cache_;
  std::array<double, 18> leg_angles_;
  std::array<double, 6> base_link_rotations_;
  std::vector<Eigen::Vector3d> base_link_translations_;

  bool joint_states_initialized_ = false;
  bool position_initialized_ = false;
  Eigen::Matrix4d odom_pose_;
  Eigen::Matrix4d previous_odom_pose_;
  Eigen::Vector3d base_footprint_;
  std::vector<Eigen::Vector3d> last_leg_positions_;
  std::shared_ptr<KinematicsSolver> solver_;
  double last_odom_estimate_time_;

  std::vector<Eigen::Vector3d> current_leg_config_;
  std::vector<Eigen::Vector3d> previous_leg_config_;
};

#endif  // ELKAPOD_ODOM_HPP
