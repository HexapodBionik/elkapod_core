#include "../include/elkapod_motion_manager/elkapod_motion_manager.hpp"

#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::literals::chrono_literals;
using namespace elkapod_motion_manager;

static std::string stateToString(const State& state) {
  if (state == State::INIT) {
    return "init";
  } else if (state == State::IDLE_LOWERED) {
    return "idle_lowered";
  } else if (state == State::IDLE) {
    return "idle";
  } else {
    return "walking";
  }
}

ElkapodMotionManager::ElkapodMotionManager() : Node("elkapod_motion_manager") {
  base_height = this->declare_parameter<double>("base_height.default_base_height");

  base_height_min = this->declare_parameter<double>("base_height.min");
  base_height_max = this->declare_parameter<double>("base_height.max");

  leg_spacing = this->declare_parameter<double>("leg_spacing.default_leg_spacing");
  leg_spacing_min = this->declare_parameter<double>("leg_spacing.min");
  leg_spacing_max = this->declare_parameter<double>("leg_spacing.max");

  leg_spacing_waypoint = this->declare_parameter<double>("standing_up.leg_spacing_waypoint");
  base_height_waypoint = this->declare_parameter<double>("standing_up.base_height_waypoint");

  // Leg mounting point correction
  base_height += 0.025;
  base_height_waypoint += 0.025;

  trajectory_freq_hz = this->declare_parameter<double>("trajectory.frequency_hz");

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diag_timer_ = this->create_timer(std::chrono::duration<double>(diag_publish_period_),
                                   std::bind(&ElkapodMotionManager::diagPublishCallback, this));

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

  switch_controller_publisher_ =
      this->create_client<controller_manager_msgs::srv::SwitchController>(
          "/controller_manager/switch_controller", 10, my_group_);

  this->leg_positions_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/elkapod_ik_controller/leg_positions", 10);

  std::chrono::duration<float> period_s{1.0f / trajectory_freq_hz};
  auto period_ms = duration_cast<std::chrono::milliseconds>(period_s);

  this->timer_ =
      this->create_timer(period_ms, std::bind(&ElkapodMotionManager::legControlCallback, this));
  this->state_ = State::INIT;

  this->planner = LinearLegPlanner();
  this->hop_planner = HopLegPlanner();
  this->executor_ = TrajectoryExecutor();
  this->executor_enable_ = false;
}

void ElkapodMotionManager::diagPublishCallback() {
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "ElkapodMotionManager: State";
  status.hardware_id = "WalkingStack";
  status.level = 0;
  status.message = "Current locomotion stack state";

  diagnostic_msgs::msg::KeyValue state_key;
  state_key.key = "state";
  state_key.value = stateToString(state_);
  status.values.push_back(state_key);

  diag_array.status.push_back(status);
  diag_pub_->publish(diag_array);
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
    auto enable_request =
        std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    enable_request->activate_asap = true;
    enable_request->activate_controllers = {"elkapod_gait_controller"};
    enable_request->strictness = 2;  // STRICT
    auto timeout = builtin_interfaces::msg::Duration();
    timeout.sec = 5;
    enable_request->timeout = timeout;

    while (!switch_controller_publisher_->wait_for_service(std::chrono::seconds(1s))) {
      if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "Interrupted. Exiting.");
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting...");
    }

    auto future = switch_controller_publisher_->async_send_request(enable_request);

    if (future.wait_for(5s) == std::future_status::ready && future.get()->ok) {
      RCLCPP_INFO(this->get_logger(), "Successfully transitioned to WALKING state");
      response->success = true;
      state_ = State::WALKING;
    } else {
      this->switch_controller_publisher_->remove_pending_request(future);
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
    auto disable_request =
        std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    disable_request->deactivate_controllers = {"elkapod_gait_controller"};
    disable_request->strictness = 2;  // STRICT
    auto timeout = builtin_interfaces::msg::Duration();
    timeout.sec = 5;
    disable_request->timeout = timeout;

    while (!switch_controller_publisher_->wait_for_service(std::chrono::seconds(1s))) {
      if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "Interrupted. Exiting.");
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting...");
    }

    auto future = switch_controller_publisher_->async_send_request(disable_request);
    if (future.wait_for(5s) == std::future_status::ready && future.get()->ok) {
      RCLCPP_INFO(this->get_logger(), "Successfully transitioned to IDLE state");
      response->success = true;
      state_ = State::IDLE;
    } else {
      switch_controller_publisher_->remove_pending_request(future);
      RCLCPP_WARN(this->get_logger(),
                  "Gait generator hasn't been disabled due to some error! "
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
  const double max_reach_x = 0.38521;
  const double movement_time_s = 4.0;

  std::array<Trajectory, 6> step_trajs;
  auto traj = hop_planner.plan({max_reach_x, 0.0, 0.025}, {leg_spacing_waypoint, 0.0, 0.0},
                               movement_time_s, trajectory_freq_hz);
  for (size_t i = 0; i < 6; ++i) {
    step_trajs[i] = traj;
  }
  trajs.push_back(step_trajs);
}

void ElkapodMotionManager::standUpPlanning() {
  const double lift_time = 1.0;
  const double leg_move_time = 0.8;
  const int steps = 5;
  const double leg_second_spacing_waypoint = 0.175;

  double spacing_step =
      (leg_spacing_waypoint - leg_second_spacing_waypoint) / static_cast<double>(steps - 1);
  if (leg_spacing >= leg_second_spacing_waypoint) {
    spacing_step = (leg_spacing_waypoint - leg_spacing) / static_cast<double>(steps - 1);
  }

  const double height_step = base_height / static_cast<double>(2 * (steps - 1));

  std::vector<double> height_waypoints;
  height_waypoints.reserve(2 * steps);
  std::generate_n(std::back_inserter(height_waypoints), 2 * steps,
                  [i = 0, height_step]() mutable { return i++ * height_step; });

  std::vector<double> spacing_waypoints;
  spacing_waypoints.reserve(steps);
  std::generate_n(
      std::back_inserter(spacing_waypoints), steps,
      [i = 0, this, spacing_step]() mutable { return leg_spacing_waypoint - i++ * spacing_step; });

  for (size_t k = 1; k < steps; ++k) {
    std::array<Trajectory, 6> step_trajs;
    auto traj = planner.plan({spacing_waypoints[k - 1], 0.0, -height_waypoints[2 * (k - 1)]},
                             {spacing_waypoints[k - 1], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                             lift_time, trajectory_freq_hz);
    for (size_t i = 0; i < 6; ++i) {
      step_trajs[i] = traj;
    }
    trajs.push_back(step_trajs);

    std::array<size_t, 6> legs_move_order = {0, 5, 2, 1, 4, 3};
    std::array<bool, 6> leg_moved = {false};

    for (size_t i = 0; i < 6; ++i) {
      for (size_t j = 0; j < 6; ++j) {
        if (legs_move_order[i] == j && !leg_moved[j]) {
          auto traj =
              hop_planner.plan({spacing_waypoints[k - 1], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                               {spacing_waypoints[k], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                               leg_move_time, trajectory_freq_hz);

          step_trajs[j] = traj;
          leg_moved[j] = true;
        } else if (legs_move_order[i] != j && leg_moved[j]) {
          auto traj = planner.plan({spacing_waypoints[k], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                                   {spacing_waypoints[k], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                                   leg_move_time, trajectory_freq_hz);
          step_trajs[j] = traj;
        } else {
          auto traj =
              planner.plan({spacing_waypoints[k - 1], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                           {spacing_waypoints[k - 1], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                           leg_move_time, trajectory_freq_hz);
          step_trajs[j] = traj;
        }
      }
      trajs.push_back(step_trajs);
    }

    auto final_traj = planner.plan({spacing_waypoints[k], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                                   {spacing_waypoints[k], 0.0, -height_waypoints[2 * k]}, lift_time,
                                   trajectory_freq_hz);

    for (size_t i = 0; i < 6; ++i) {
      step_trajs[i] = final_traj;
    }
    trajs.push_back(step_trajs);
  }

  if (leg_spacing < leg_second_spacing_waypoint) {
    const int second_spacing_steps = 3;
    const double second_spacing_step =
        (leg_second_spacing_waypoint - leg_spacing) / static_cast<double>(second_spacing_steps - 1);
    std::vector<double> second_spacing_waypoints;
    second_spacing_waypoints.reserve(second_spacing_steps);
    std::generate_n(std::back_inserter(second_spacing_waypoints), second_spacing_steps,
                    [i = 0, leg_second_spacing_waypoint, second_spacing_step]() mutable {
                      return leg_second_spacing_waypoint - i++ * second_spacing_step;
                    });

    std::array<size_t, 6> legs_move_order = {0, 5, 2, 1, 4, 3};
    for (size_t k = 1; k < second_spacing_steps; ++k) {
      std::array<Trajectory, 6> step_trajs;

      std::array<bool, 6> leg_moved = {false};

      for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < 6; ++j) {
          if (legs_move_order[i] == j && !leg_moved[j]) {
            auto traj = hop_planner.plan({second_spacing_waypoints[k - 1], 0.0, -base_height},
                                         {second_spacing_waypoints[k], 0.0, -base_height},
                                         leg_move_time, trajectory_freq_hz);

            step_trajs[j] = traj;
            leg_moved[j] = true;
          } else if (legs_move_order[i] != j && leg_moved[j]) {
            auto traj = planner.plan({second_spacing_waypoints[k], 0.0, -base_height},
                                     {second_spacing_waypoints[k], 0.0, -base_height},
                                     leg_move_time, trajectory_freq_hz);
            step_trajs[j] = traj;
          } else {
            auto traj = planner.plan({second_spacing_waypoints[k - 1], 0.0, -base_height},
                                     {second_spacing_waypoints[k - 1], 0.0, -base_height},
                                     leg_move_time, trajectory_freq_hz);
            step_trajs[j] = traj;
          }
        }
        trajs.push_back(step_trajs);
      }
    }
  }
}

void ElkapodMotionManager::lowerDownPlanning() {
  const double lift_time = 1.0;
  const double leg_move_time = 0.8;
  const int steps = 5;
  const double spacing_step = (leg_spacing_waypoint - leg_spacing) / static_cast<double>(steps - 1);
  const double height_step = (base_height) / static_cast<double>(2 * (steps - 1));

  std::vector<double> height_waypoints;
  height_waypoints.reserve(2 * steps);
  std::generate_n(std::back_inserter(height_waypoints), 2 * steps,
                  [i = 0, this, height_step]() mutable { return base_height - i++ * height_step; });

  std::vector<double> spacing_waypoints;
  spacing_waypoints.reserve(steps);
  std::generate_n(
      std::back_inserter(spacing_waypoints), steps,
      [i = 0, this, spacing_step]() mutable { return leg_spacing + i++ * spacing_step; });

  for (size_t k = 1; k < steps; ++k) {
    std::array<Trajectory, 6> step_trajs;
    auto traj = planner.plan({spacing_waypoints[k - 1], 0.0, -height_waypoints[2 * (k - 1)]},
                             {spacing_waypoints[k - 1], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                             lift_time, trajectory_freq_hz);
    for (size_t i = 0; i < 6; ++i) {
      step_trajs[i] = traj;
    }
    trajs.push_back(step_trajs);

    std::array<size_t, 6> legs_move_order = {3, 4, 1, 2, 5, 0};
    std::array<bool, 6> leg_moved = {false};

    for (size_t i = 0; i < 6; ++i) {
      for (size_t j = 0; j < 6; ++j) {
        if (legs_move_order[i] == j && !leg_moved[j]) {
          auto traj =
              hop_planner.plan({spacing_waypoints[k - 1], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                               {spacing_waypoints[k], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                               leg_move_time, trajectory_freq_hz);

          step_trajs[j] = traj;
          leg_moved[j] = true;
        } else if (legs_move_order[i] != j && leg_moved[j]) {
          auto traj = planner.plan({spacing_waypoints[k], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                                   {spacing_waypoints[k], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                                   leg_move_time, trajectory_freq_hz);
          step_trajs[j] = traj;
        } else {
          auto traj =
              planner.plan({spacing_waypoints[k - 1], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                           {spacing_waypoints[k - 1], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                           leg_move_time, trajectory_freq_hz);
          step_trajs[j] = traj;
        }
      }
      trajs.push_back(step_trajs);
    }

    auto final_traj = planner.plan({spacing_waypoints[k], 0.0, -height_waypoints[2 * (k - 1) + 1]},
                                   {spacing_waypoints[k], 0.0, -height_waypoints[2 * k]}, lift_time,
                                   trajectory_freq_hz);

    for (size_t i = 0; i < 6; ++i) {
      step_trajs[i] = final_traj;
    }
    trajs.push_back(step_trajs);
  }
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
