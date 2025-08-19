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
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <unordered_map>

#include "leg_path.hpp"

namespace elkapod_gait_gen {
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
  enum class State { IDLE = 0, WALKING = 1, DISABLED = 2 };
  enum class GaitType { WAVE = 0, RIPPLE = 1, TRIPOD = 2 };
  inline static constexpr size_t kLegsNb = 6;
  inline static constexpr size_t kJointsNum = 18;

  void init(), deinit();

  // Callbacks
  void enableServiceCallback(ServiceTriggerSrv_Req request, ServiceTriggerSrv_Resp response);
  void disableServiceCallback(ServiceTriggerSrv_Req request, ServiceTriggerSrv_Resp response);
  void paramCallback(const FloatMsg::SharedPtr msg);
  void gaitTypeCallback(const IntMsg::SharedPtr msg);
  void velocityCallback(const VelCmd::SharedPtr msg);

  // Gait logic
  void changeGait();
  void clockFunction(double t, double T, double phase_shift, int leg_nb);
  void updateVelocityCommand();
  void updateAndWriteCommands();

  void velocityDeadzone(Eigen::Vector2d& linear_vel, double& angular_vel);
  void velocityClamp(Eigen::Vector2d& linear_vel, double& angular_vel);

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr leg_clock_timer_;
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr leg_signal_pub_, leg_phase_pub_;
  rclcpp::Subscription<VelCmd>::SharedPtr velocity_sub_;
  rclcpp::Subscription<FloatMsg>::SharedPtr param_sub_;
  rclcpp::Subscription<IntMsg>::SharedPtr gait_type_sub_;
  rclcpp::Service<ServiceTriggerSrv>::SharedPtr enable_srv_, disable_srv_;

  // Variables
  State state_ = State::DISABLED;
  GaitType gait_type_ = GaitType::RIPPLE;
  double trajectory_freq_hz, min_swing_time_sec_;
  double leg_spacing_, step_length_, step_height_, phase_lag_;
  double set_base_height_, cycle_time_, duty_factor_;
  double current_vel_scalar_, current_angular_velocity_;

  double base_height_;
  double base_height_min_;
  double base_height_max_;

  VelCmd received_vel_command_;
  Eigen::Vector2d current_vel_command_;

  std::unordered_map<GaitType, double> max_vel_dict_ = {
      {GaitType::TRIPOD, 0.4}, {GaitType::RIPPLE, 0.2}, {GaitType::WAVE, 0.05}};

  // TODO to be validated later
  std::unordered_map<GaitType, double> max_angular_vel_dict_ = {
      {GaitType::TRIPOD, 0.9}, {GaitType::RIPPLE, 0.9}, {GaitType::WAVE, 0.9}};

  std::unordered_map<GaitType, double> cycle_time_dict_ = {
      {GaitType::TRIPOD, 1.2}, {GaitType::RIPPLE, 1.5}, {GaitType::WAVE, 4.0}};

  double max_vel_ = 0;
  double max_angular_vel_ = 0;

  // Temporary parameters
  const double deadzone_d_ = 0.003;
  double ema_filter_alfa_ = 0.0;
  double handover_sec_ = 0.1;

  rclcpp::Time init_time_;
  std::vector<Eigen::Vector2d> current_velocity_;
  std::vector<Eigen::Vector3d> last_leg_position_;
  std::vector<double> phase_offset_, leg_phase_shift_;
  std::vector<int> leg_phase_;
  std::vector<double> leg_clock_;
  std::array<double, kLegsNb> base_link_rotations_;
  std::vector<Eigen::Vector3d> base_link_translations_;
  std::unique_ptr<ElkapodLegPath> base_traj_;
};
};  // namespace elkapod_gait_gen

#endif  // ELKAPOD_GAIT_GEN_HPP
