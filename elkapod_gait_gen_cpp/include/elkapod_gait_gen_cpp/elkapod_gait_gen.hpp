//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef ELKAPOD_GAIT_GEN_HPP
#define ELKAPOD_GAIT_GEN_HPP

#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "leg_path.hpp"

using namespace std::chrono_literals;

enum class State { IDLE = 0, WALKING = 1, DISABLED = 2 };

enum class GaitType { WAVE = 0, TRIPOID = 1 };

using namespace std::chrono_literals;
using ServiceTriggerSrv = std_srvs::srv::Trigger;
using ServiceTriggerSrv_Req = std::shared_ptr<std_srvs::srv::Trigger_Request>;
using ServiceTriggerSrv_Resp = std::shared_ptr<std_srvs::srv::Trigger_Response>;
using FloatArrayMsg = std_msgs::msg::Float64MultiArray;
using FloatMsg = std_msgs::msg::Float64;
using IntMsg = std_msgs::msg::Int32;
using VelCmd = geometry_msgs::msg::Twist;

class ElkapodGaitGen : public rclcpp::Node {
 public:
  ElkapodGaitGen();

 private:
  void init(), deinit();

  // Callbacks
  void enableServiceCallback(ServiceTriggerSrv_Req request, ServiceTriggerSrv_Resp response);
  void disableServiceCallback(ServiceTriggerSrv_Req request, ServiceTriggerSrv_Resp response);
  void paramCallback(const FloatMsg::SharedPtr msg);
  void gaitTypeCallback(const IntMsg::SharedPtr msg);
  void velocityCallback(const VelCmd::SharedPtr msg);
  void legClockCallback();

  // Gait logic
  void changeGait(GaitType gait_type);
  void clockFunction(double t, double T, double phase_shift, int leg_nb);

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr leg_clock_timer_;
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr leg_signal_pub_, leg_phase_pub_;
  rclcpp::Subscription<VelCmd>::SharedPtr velocity_sub_;
  rclcpp::Subscription<FloatMsg>::SharedPtr param_sub_;
  rclcpp::Subscription<IntMsg>::SharedPtr gait_type_sub_;
  rclcpp::Service<ServiceTriggerSrv>::SharedPtr enable_srv_, disable_srv_;

  // Variables
  State state_ = State::DISABLED;
  GaitType gait_type_ = GaitType::TRIPOID;
  double trajectory_freq_hz, min_swing_time_sec_;
  double leg_spacing_, step_length_, step_height_, phase_lag_;
  double set_base_height_, cycle_time_, swing_percentage_;
  double current_vel_scalar_, current_angular_velocity_, current_base_direction_;

  double base_height_;
  double base_height_min_;
  double base_height_max_;

  rclcpp::Time init_time_;
  Eigen::Vector3d current_vel_;
  std::vector<Eigen::Vector2d> current_velocity_;
  std::vector<Eigen::Vector3d> last_leg_position_;
  std::vector<double> phase_offset_, leg_phase_shift_;
  std::vector<int> leg_phase_;
  std::vector<double> leg_clock_;
  std::array<double, 6> base_link_rotations_;
  std::vector<Eigen::Vector3d> base_link_translations_;
  std::unique_ptr<ElkapodLegPath> base_traj_;
};

#endif  // ELKAPOD_GAIT_GEN_HPP
