

//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef ELKAPOD_MOTION_MANAGER_HPP
#define ELKAPOD_MOTION_MANAGER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <semaphore>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "elkapod_leg_trajectory.hpp"
#include "elkapod_msgs/action/motion_manager_trigger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

typedef enum { INIT, IDLE_LOWERED, IDLE, WALKING } State;

class ElkapodMotionManager : public rclcpp::Node {
 public:
  using TriggerAction = elkapod_msgs::action::MotionManagerTrigger;
  using GoalHandleTriggerAction = rclcpp_action::ServerGoalHandle<TriggerAction>;

  ElkapodMotionManager();
  void initNode();

 private:
  void lowerDownPlanning();
  void standUpPlanning();
  void initPlanning();

  rclcpp::CallbackGroup::SharedPtr my_group_;

  rclcpp_action::GoalResponse transition_action_handle_goal(
      const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const TriggerAction::Goal> goal);
  void transition_action_handle_accepted(
      const std::shared_ptr<GoalHandleTriggerAction> goal_handle);
  void transition_action_execute(const std::shared_ptr<GoalHandleTriggerAction> goal_handle);

  void walkEnableServiceCallback(std::shared_ptr<std_srvs::srv::Trigger_Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger_Response> response);
  void walkDisableServiceCallback(std::shared_ptr<std_srvs::srv::Trigger_Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger_Response> response);

  void legControlCallback();

  rclcpp_action::Server<TriggerAction>::SharedPtr transition_action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr walk_enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr walk_disable_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gait_enable_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gait_disable_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_positions_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  State state_;
  State next_state_;
  std::function<void()> planning_method_;
  std::binary_semaphore semaphore_{0};

  std::vector<std::array<Trajectory, 6>> trajs;
  LinearLegPlanner planner;
  HopLegPlanner hop_planner;
  TrajectoryExecutor executor_;
  bool executor_enable_;

  // Standing up variables
  double base_height_waypoint;
  double leg_spacing_waypoint;

  // Height variables
  double base_height;
  double base_height_min;
  double base_height_max;

  // Leg spacing variables
  double leg_spacing;
  double leg_spacing_min;
  double leg_spacing_max;

  // Trajectory variables
  double trajectory_freq_hz;
};

#endif  // ELKAPOD_MOTION_MANAGER_HPP
