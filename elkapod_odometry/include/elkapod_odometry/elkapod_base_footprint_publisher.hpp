//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef ELKAPOD_BASE_FOOTPRINT_PUBLISHER_HPP
#define ELKAPOD_BASE_FOOTPRINT_PUBLISHER_HPP

#include <array>
#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "elkapod_core_lib/leg_kinematics.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW

using namespace std::chrono_literals;
using KinematicsSolver = elkapod_core_lib::kinematics::KinematicsSolver;

class ElkapodBaseFootprintPublisher : public rclcpp::Node {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ElkapodBaseFootprintPublisher();

 private:
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr joint_states);
  void computeBaseFootprintCallback();
  void tfCallback();

  Eigen::Vector4d findPlane(const Eigen::Matrix3Xd contact_points);
  Eigen::Vector3d findBaseFootprintCoords(Eigen::Vector4d plane);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr broadcaster_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::array<double, 18> leg_angles_;
  std::array<double, 6> base_link_rotations_;
  std::vector<Eigen::Vector3d> base_link_translations_;

  bool joint_states_initialized_ = false;
  bool position_initialized_ = false;
  Eigen::Vector3d base_footprint_;
  // Orientation of base_footprint in base_link, z axis along the support plane normal
  Eigen::Quaterniond base_footprint_rotation_ = Eigen::Quaterniond::Identity();
  std::shared_ptr<KinematicsSolver> solver_;
};

#endif  // ELKAPOD_BASE_FOOTPRINT_PUBLISHER_HPP
