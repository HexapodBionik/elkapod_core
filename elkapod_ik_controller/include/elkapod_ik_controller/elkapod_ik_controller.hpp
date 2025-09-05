

//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef ELKAPOD_LEG_CONTROLLER_HPP
#define ELKAPOD_LEG_CONTROLLER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

#include "controller_interface/chainable_controller_interface.hpp"
#include "elkapod_ik_controller/elkapod_ik_controller_parameters.hpp"
#include "elkapod_leg_inverse_kinematics.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

namespace elkapod_ik_controller {
class ElkapodIKController : public controller_interface::ChainableControllerInterface {
 public:
  ElkapodIKController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Chainable controller replaces update() with the following two functions
  controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::return_type update_and_write_commands(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

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

 protected:
  bool on_set_chained_mode(bool chained_mode) override;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_subscriber_ =
      nullptr;

  realtime_tools::RealtimeThreadSafeBox<std_msgs::msg::Float64MultiArray> received_position_msg_;
  std_msgs::msg::Float64MultiArray command_msg_;

  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  rclcpp::Time previous_update_timestamp_{0};

  std::shared_ptr<KinematicsSolver> solver_;

  bool reset();
  void halt();

 private:
  void reset_buffers();
};

}  // namespace elkapod_ik_controller

#endif  // ELKAPOD_LEG_CONTROLLER_HPP
