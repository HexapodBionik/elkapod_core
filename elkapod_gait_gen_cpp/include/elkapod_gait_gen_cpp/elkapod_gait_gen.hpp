//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef ELKAPOD_GAIT_GEN_HPP
#define ELKAPOD_GAIT_GEN_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>

#include "elkapod_core_lib/control.hpp"
#include "leg_path.hpp"

namespace elkapod_gait_gen {
using ServiceTriggerSrv = std_srvs::srv::Trigger;
using ServiceTriggerSrv_Req = std::shared_ptr<std_srvs::srv::Trigger_Request>;
using ServiceTriggerSrv_Resp = std::shared_ptr<std_srvs::srv::Trigger_Response>;
using FloatArrayMsg = std_msgs::msg::Float64MultiArray;
using Int8ArrayMsg = std_msgs::msg::Int8MultiArray;
using FloatMsg = std_msgs::msg::Float64;
using IntMsg = std_msgs::msg::Int32;
using VelCmd = geometry_msgs::msg::Twist;
using IMUMsg = sensor_msgs::msg::Imu;
using PID = elkapod_core_lib::control::PID;

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
  void imuCallback(const IMUMsg::SharedPtr msg);
  void rollCallback(const FloatMsg::SharedPtr msg);
  void pitchCallback(const FloatMsg::SharedPtr msg);
  void fsrCallback(const Int8ArrayMsg::SharedPtr msg);

  // Gait logic
  void changeGait();
  void clockFunction(double t, double T, double phase_shift, int leg_nb);
  void updateVelocityCommand();
  void updateAndWriteCommands();

  void velocityDeadzone(Eigen::Vector2d& linear_vel, double& angular_vel);
  void velocityClamp(Eigen::Vector2d& linear_vel, double& angular_vel);

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr leg_clock_timer_;
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr leg_signal_pub_;
  rclcpp::Subscription<VelCmd>::SharedPtr velocity_sub_;
  rclcpp::Subscription<FloatMsg>::SharedPtr param_sub_;
  rclcpp::Subscription<IMUMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<FloatMsg>::SharedPtr roll_sub_;
  rclcpp::Subscription<FloatMsg>::SharedPtr pitch_sub_;
  rclcpp::Subscription<IntMsg>::SharedPtr gait_type_sub_;
  rclcpp::Subscription<Int8ArrayMsg>::SharedPtr fsr_sub_;
  rclcpp::Service<ServiceTriggerSrv>::SharedPtr enable_srv_, disable_srv_;

  // Variables
  State state_ = State::DISABLED;
  GaitType gait_type_ = GaitType::TRIPOD;
  double trajectory_freq_hz, min_swing_time_sec_;
  double leg_spacing_, step_length_, step_height_, phase_lag_;
  double set_base_height_, cycle_time_, duty_factor_;
  double current_vel_scalar_, current_angular_velocity_;

  double base_height_;
  double base_height_min_;
  double base_height_max_;
  double base_height_ema_filter_alfa_ = 0.0;

  VelCmd received_vel_command_;
  Eigen::Vector2d current_vel_command_;

  std::unordered_map<GaitType, double> max_vel_dict_ = {
      {GaitType::TRIPOD, 0.2}, {GaitType::RIPPLE, 0.15}, {GaitType::WAVE, 0.05}};

  // TODO to be validated later
  std::unordered_map<GaitType, double> max_angular_vel_dict_ = {
      {GaitType::TRIPOD, 0.5}, {GaitType::RIPPLE, 0.5}, {GaitType::WAVE, 0.5}};

  std::unordered_map<GaitType, double> cycle_time_dict_ = {
      {GaitType::TRIPOD, 1.2}, {GaitType::RIPPLE, 1.2}, {GaitType::WAVE, 2.0}};

  double max_vel_ = 0;
  double max_angular_vel_ = 0;
  const double deadzone_d_ = 0.003;
  double ema_filter_alfa_ = 0.0;

  rclcpp::Time init_time_;
  std::vector<Eigen::Vector2d> current_velocity_;
  std::vector<Eigen::Vector3d> last_leg_position_;
  std::vector<Eigen::Vector3d> last_leg_position_relative_;
  std::vector<double> phase_offset_, leg_phase_shift_;
  std::vector<int> leg_phase_;
  std::vector<double> leg_clock_;
  std::array<double, kLegsNb> base_link_rotations_;
  std::vector<Eigen::Vector3d> base_link_translations_;
  std::unique_ptr<elkapod_leg_paths::BasicPathBezier> leg_path_gen_;

  PID roll_pid_, pitch_pid_;
  tf2::Quaternion q_;
  double roll_, pitch_, yaw_;
  double set_roll_ = 0.0;
  double set_pitch_ = 0.0;
  double roll_limit_ = 0.0;
  double pitch_limit_ = 0.0;

  std::vector<double> fsr_data_;
};
};  // namespace elkapod_gait_gen

#endif  // ELKAPOD_GAIT_GEN_HPP
