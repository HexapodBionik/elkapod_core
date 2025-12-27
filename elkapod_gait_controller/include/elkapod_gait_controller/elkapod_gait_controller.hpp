//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef ELKAPOD_LEG_CONTROLLER_HPP
#define ELKAPOD_LEG_CONTROLLER_HPP

#include <builtin_interfaces/msg/duration.hpp>
#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>

#include "control_toolbox/pid_ros.hpp"
#include "controller_interface/controller_interface.hpp"
#include "elkapod_gait_controller/elkapod_gait_controller_parameters.hpp"
#include "leg_path.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

namespace elkapod_gait_controller {
using FloatArrayMsg = std_msgs::msg::Float64MultiArray;
using FloatMsg = std_msgs::msg::Float64;
using IntMsg = std_msgs::msg::Int32;
using VelCmd = geometry_msgs::msg::Twist;
using IMUMsg = sensor_msgs::msg::Imu;
using DurationMsg = builtin_interfaces::msg::Duration;

class ElkapodGaitController : public controller_interface::ControllerInterface {
 public:
  ElkapodGaitController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  bool reset();
  void halt();
  void reset_buffers();

  enum class State { IDLE = 0, WALKING = 1, DISABLED = 2 };
  enum class GaitType { WAVE = 0, RIPPLE = 1, TRIPOD = 2 };
  inline static constexpr size_t kLegsNb = 6;
  inline static constexpr size_t kJointsNum = 18;

  // Callbacks
  void baseHeightCallback(const FloatMsg::SharedPtr msg);
  void gaitTypeCallback(const IntMsg::SharedPtr msg);
  void velocityCallback(const VelCmd::SharedPtr msg);
  void imuCallback(const IMUMsg::SharedPtr msg);
  void rollCallback(const FloatMsg::SharedPtr msg);
  void pitchCallback(const FloatMsg::SharedPtr msg);

  // Gait logic
  void changeGait();
  void clockFunction(double t, double T, double phase_shift, int leg_nb);
  void updateVelocityCommand();

  void velocityDeadzone(Eigen::Vector2d& vel, double& angular_vel);
  void velocityClamp(Eigen::Vector2d& vel, double& angular_vel);

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr leg_clock_timer_;
  rclcpp::Subscription<VelCmd>::SharedPtr velocity_sub_;
  rclcpp::Subscription<FloatMsg>::SharedPtr param_sub_;
  rclcpp::Subscription<IMUMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<FloatMsg>::SharedPtr roll_sub_;
  rclcpp::Subscription<FloatMsg>::SharedPtr pitch_sub_;
  rclcpp::Subscription<IntMsg>::SharedPtr gait_type_sub_;

  // Variables
  State state_ = State::DISABLED;
  GaitType gait_type_ = GaitType::TRIPOD;
  double min_swing_time_sec_;
  double leg_spacing_, step_length_, step_height_, phase_lag_;
  double set_base_height_, cycle_time_, duty_factor_;
  double current_vel_scalar_, current_angular_velocity_;

  double default_base_height_;
  double base_height_offset_;
  std::vector<double> kinematics_m1_;
  double base_height_;
  double base_height_min_;
  double base_height_max_;
  double base_height_ema_filter_alfa_ = 0.0;

  realtime_tools::RealtimeThreadSafeBox<VelCmd> input_vel_command_;

  VelCmd received_vel_command_;
  Eigen::Vector2d current_vel_command_;

  std::unordered_map<GaitType, double> max_vel_dict_;
  std::unordered_map<GaitType, double> max_angular_vel_dict_;

  std::unordered_map<GaitType, double> cycle_time_dict_ = {
      {GaitType::TRIPOD, 1.2}, {GaitType::RIPPLE, 1.2}, {GaitType::WAVE, 2.0}};

  double max_vel_ = 0;
  double max_angular_vel_ = 0;
  const double deadzone_d_ = 0.003;
  double ema_filter_alfa_ = 0.0;

  rclcpp::Time init_time_;
  rclcpp::Time last_time_;
  std::vector<Eigen::Vector2d> current_velocity_;
  std::vector<Eigen::Vector3d> last_leg_position_;
  std::vector<Eigen::Vector3d> last_leg_position_relative_;
  std::vector<double> phase_offset_, leg_phase_shift_;
  std::vector<int> leg_phase_;
  std::vector<double> leg_clock_;
  std::array<double, kLegsNb> base_link_rotations_;
  std::vector<Eigen::Vector3d> base_link_translations_;
  std::unique_ptr<elkapod_leg_paths::BasicPathBezier> leg_path_gen_;

  std::unique_ptr<control_toolbox::Pid> roll_pid_;
  std::unique_ptr<control_toolbox::Pid> pitch_pid_;

  tf2::Quaternion q_;
  double roll_, pitch_, yaw_;
  double set_roll_ = 0.0;
  double set_pitch_ = 0.0;
  double roll_limit_ = 0.0;
  double pitch_limit_ = 0.0;

  bool configured_ = false;

  // Debug info
  std::unique_ptr<realtime_tools::RealtimePublisher<DurationMsg>> loop_exec_duration_publisher_rt_;
  rclcpp::Publisher<DurationMsg>::SharedPtr loop_exec_duration_publisher_;
  rclcpp::Time last_duration_msg_publish_time_;
  bool publish_loop_execution_time_ = false;
  inline static constexpr double duration_msg_publish_period_ = 0.05;  // 50 Hz
  bool is_first_update_ = true;
};

}  // namespace elkapod_gait_controller

#endif  // ELKAPOD_LEG_CONTROLLER_HPP
