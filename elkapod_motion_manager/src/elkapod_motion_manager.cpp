#include "../include/elkapod_motion_manager/elkapod_motion_manager.hpp"

#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::literals::chrono_literals;

ElkapodMotionManager::ElkapodMotionManager() : Node("elkapod_motion_manager") {
  base_height = this->declare_parameter<double>("../common_config.base_height.default", 0.17);
  base_height_min = this->declare_parameter<double>("../common_config.base_height.min", 0.12);
  base_height_max = this->declare_parameter<double>("../common_config.base_height.max", 0.22);

  leg_spacing = this->declare_parameter<double>("../common_config.leg_spacing.default", 0.175);
  leg_spacing_min = this->declare_parameter<double>("../common_config.leg_spacing.min", 0.1);
  leg_spacing_max = this->declare_parameter<double>("../common_config.leg_spacing.max", 0.25);

  leg_spacing_waypoint = this->declare_parameter<double>("standing_up.leg_spacing_waypoint", 0.25);
  base_height_waypoint = this->declare_parameter<double>("standing_up.base_height_waypoint", 0.1);

  trajectory_freq_hz = this->declare_parameter<double>("trajectory.frequency_hz", 20);

  this->transition_action_server_ = rclcpp_action::create_server<TriggerAction>(
      this, "motion_manager_transition",
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const TriggerAction::Goal> goal) {
        return this->transition_action_handle_goal(uuid, goal);
      },
      nullptr,
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<TriggerAction>> goal_handle) {
        this->transition_action_handle_accepted(goal_handle);
      });

  my_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  this->walk_enable_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/motion_manager_walk_enable",
      std::bind(&ElkapodMotionManager::walkEnableServiceCallback, this, _1, _2), 10, my_group_);
  this->walk_disable_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/motion_manager_walk_disable",
      std::bind(&ElkapodMotionManager::walkDisableServiceCallback, this, _1, _2), 10, my_group_);

  this->gait_enable_publisher_ =
      this->create_client<std_srvs::srv::Trigger>("/gait_gen_enable", 10, my_group_);
  this->gait_disable_publisher_ =
      this->create_client<std_srvs::srv::Trigger>("/gait_gen_disable", 10, my_group_);

  this->leg_positions_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/elkapod_leg_positions", 10);

   std::chrono::duration<float> period_s{1.0f / trajectory_freq_hz};
    auto period_ms = duration_cast<std::chrono::milliseconds>(period_s);

  this->timer_ =
      this->create_wall_timer(period_ms, std::bind(&ElkapodMotionManager::legControlCallback, this));
  this->state_ = State::INIT;

  this->planner = LinearLegPlanner();
  this->hop_planner = HopLegPlanner();
  this->executor_ = TrajectoryExecutor();
  this->executor_enable_ = false;
}

void ElkapodMotionManager::initNode() {
  RCLCPP_INFO(this->get_logger(), "Elkapod motion manager initialized!");
}

rclcpp_action::GoalResponse ElkapodMotionManager::transition_action_handle_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const TriggerAction::Goal> goal) {
  if (executor_enable_) {
    RCLCPP_INFO(this->get_logger(),
                "Robot is currently transitioning to next state thus new goal "
                "has been rejected!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->transition == "init" && state_ == State::INIT) {
    RCLCPP_INFO(this->get_logger(), "Init goal accepted!");
    planning_method_ = std::bind(&ElkapodMotionManager::initPlanning, this);
    next_state_ = State::IDLE_LOWERED;
  } else if (goal->transition == "stand_up" && state_ == State::IDLE_LOWERED) {
    RCLCPP_INFO(this->get_logger(), "Stand up goal accepted!");
    planning_method_ = std::bind(&ElkapodMotionManager::standUpPlanning, this);
    next_state_ = State::IDLE;
  } else if (goal->transition == "lower" && state_ == State::IDLE) {
    RCLCPP_INFO(this->get_logger(), "Lower goal accepted!");
    planning_method_ = std::bind(&ElkapodMotionManager::lowerDownPlanning, this);
    next_state_ = State::IDLE_LOWERED;
  } else {  // Reject the goal
    RCLCPP_INFO(this->get_logger(), "Goal rejected!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ElkapodMotionManager::transition_action_handle_accepted(
    const std::shared_ptr<GoalHandleTriggerAction> goal_handle) {
  auto execute_in_thread = [this, goal_handle]() {
    return this->transition_action_execute(goal_handle);
  };
  std::thread{execute_in_thread}.detach();
}

void ElkapodMotionManager::transition_action_execute(
    const std::shared_ptr<GoalHandleTriggerAction> goal_handle) {
  planning_method_();
  auto current_traj = trajs.front();
  trajs.erase(trajs.begin());
  executor_.setTrajectories(current_traj);
  executor_enable_ = true;

  semaphore_.acquire();
  state_ = next_state_;

  auto result = std::make_shared<TriggerAction::Result>();
  result->success = true;

  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

void ElkapodMotionManager::walkEnableServiceCallback(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
    std::shared_ptr<std_srvs::srv::Trigger_Response> response) {
  if (!executor_enable_ && state_ == State::IDLE) {
    auto gait_request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (!this->gait_enable_publisher_->wait_for_service(std::chrono::seconds(1s))) {
      if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "Interrupted. Exiting.");
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting...");
    }

    auto future = this->gait_enable_publisher_->async_send_request(gait_request);

    if (future.wait_for(5s) == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Successfully transitioned to WALKING state");
      response->success = true;
      state_ = State::WALKING;
    } else {
      this->gait_enable_publisher_->remove_pending_request(future);
      RCLCPP_WARN(this->get_logger(),
                  "Gait generator hasn't been enabled due to some error! "
                  "Staying in IDLE state.");
      response->success = false;
    }
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Can't transition to WALKING state because ongoing transition "
                "and/or manager is in state which prevents such transition");
    response->success = false;
  }
}

void ElkapodMotionManager::walkDisableServiceCallback(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
    std::shared_ptr<std_srvs::srv::Trigger_Response> response) {
  if (!executor_enable_ && state_ == State::WALKING) {
    auto gait_request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (!this->gait_disable_publisher_->wait_for_service(std::chrono::seconds(1s))) {
      if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "Interrupted. Exiting.");
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting...");
    }

    auto future = this->gait_disable_publisher_->async_send_request(gait_request);
    if (future.wait_for(5s) == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Successfully transitioned to IDLE state");
      response->success = true;
      state_ = State::IDLE;
    } else {
      this->gait_disable_publisher_->remove_pending_request(future);
      RCLCPP_WARN(this->get_logger(),
                  "Gait generator hasn't been enabled due to some error! "
                  "Staying in WALKING state.");
      response->success = false;
    }
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Can't transition to IDLE state because ongoing transition "
                "and/or manager is in state which prevents such transition");
    response->success = false;
  }
}

void ElkapodMotionManager::initPlanning() {
  const double max_reach_x = 0.38;
  const double movement_time_s = 10;

  std::array<Trajectory, 6> step_trajs;
  for (int i = 0; i < 6; ++i) {
    auto traj = hop_planner.plan({max_reach_x, 0.0, 0.0}, {leg_spacing_waypoint, 0.0, 0.0},
                                 movement_time_s, trajectory_freq_hz);
    step_trajs[i] = traj;
  }
  trajs.push_back(step_trajs);
}

void ElkapodMotionManager::standUpPlanning() {
  std::array<Trajectory, 6> step_trajs;
  // First step - lift up a little bit
  for (int i = 0; i < 6; ++i) {
    auto traj =
        planner.plan({leg_spacing_waypoint, 0.0, 0.0},
                     {leg_spacing_waypoint, 0.0, -base_height_waypoint}, 10, trajectory_freq_hz);
    step_trajs[i] = traj;
  }
  trajs.push_back(step_trajs);

  // Second step - hop each leg one at a time
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (i == j) {
        auto traj =
            hop_planner.plan({leg_spacing_waypoint, 0.0, -base_height_waypoint},
                             {leg_spacing, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
        step_trajs[j] = traj;
      } else if (j < i) {
        auto traj = planner.plan({leg_spacing, 0.0, -base_height_waypoint},
                                 {leg_spacing, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
        step_trajs[j] = traj;
      } else {
        auto traj =
            planner.plan({leg_spacing_waypoint, 0.0, -base_height_waypoint},
                         {leg_spacing_waypoint, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
        step_trajs[j] = traj;
      }
    }
    trajs.push_back(step_trajs);
  }

  // Final lift up
  for (int i = 0; i < 6; ++i) {
    auto traj = planner.plan({leg_spacing, 0.0, -base_height_waypoint},
                             {leg_spacing, 0.0, -base_height}, 10, trajectory_freq_hz);
    step_trajs[i] = traj;
  }
  trajs.push_back(step_trajs);
}

void ElkapodMotionManager::lowerDownPlanning() {
  std::array<Trajectory, 6> step_trajs;

  for (int i = 0; i < 6; ++i) {
    auto traj = planner.plan({leg_spacing, 0.0, -base_height},
                             {leg_spacing, 0.0, -base_height_waypoint}, 10, trajectory_freq_hz);
    step_trajs[i] = traj;
  }
  trajs.push_back(step_trajs);

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (i == j) {
        auto traj = hop_planner.plan({leg_spacing, 0.0, -base_height_waypoint},
                                     {leg_spacing_waypoint, 0.0, -base_height_waypoint}, 2,
                                     trajectory_freq_hz);
        step_trajs[j] = traj;
      } else if (j < i) {
        auto traj =
            planner.plan({leg_spacing_waypoint, 0.0, -base_height_waypoint},
                         {leg_spacing_waypoint, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
        step_trajs[j] = traj;
      } else {
        auto traj = planner.plan({leg_spacing, 0.0, -base_height_waypoint},
                                 {leg_spacing, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
        step_trajs[j] = traj;
      }
    }
    trajs.push_back(step_trajs);
  }

  for (int i = 0; i < 6; ++i) {
    auto traj = planner.plan({leg_spacing_waypoint, 0.0, -base_height_waypoint},
                             {leg_spacing_waypoint, 0.0, 0.0}, 10, trajectory_freq_hz);
    step_trajs[i] = traj;
  }
  trajs.push_back(step_trajs);
}

void ElkapodMotionManager::legControlCallback() {
  std_msgs::msg::Float64MultiArray output_msg;
  output_msg.data.resize(18);

  if (executor_enable_ && !executor_.hasNext() && trajs.size() > 0) {
    auto current_traj = trajs.front();
    trajs.erase(trajs.begin());
    executor_.setTrajectories(current_traj);
    std::string msg2 = "Trajectories left: " + std::to_string(trajs.size());
    RCLCPP_INFO(this->get_logger(), msg2.c_str());
  }

  if (executor_enable_ && (executor_.hasNext() || trajs.size() > 0)) {
    auto step = executor_.next();
    size_t index = 0;
    for (const auto& leg : step) {
      for (double coordinate : leg) {
        output_msg.data[index++] = coordinate;
      }
    }
    this->leg_positions_pub_->publish(output_msg);
  } else if (executor_enable_ && !executor_.hasNext() && trajs.size() <= 0) {
    executor_enable_ = false;
    semaphore_.release();
  }
}
