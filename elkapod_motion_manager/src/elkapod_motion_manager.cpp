#include "../include/elkapod_motion_manager/elkapod_motion_manager.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

ElkapodMotionManager::ElkapodMotionManager(): Node("elkapod_motion_manager"){
    base_height = this->declare_parameter<double>("base_height.default", 0.17);
    base_height_min = this->declare_parameter<double>("base_height.min", 0.12);
    base_height_max = this->declare_parameter<double>("base_height.max", 0.22);

    leg_spacing = this->declare_parameter<double>("leg_spacing.default", 0.175);
    leg_spacing_min = this->declare_parameter<double>("leg_spacing.min", 0.1);
    leg_spacing_max = this->declare_parameter<double>("leg_spacing.max", 0.25);

    leg_spacing_waypoint = this->declare_parameter<double>("standing_up.leg_spacing_waypoint", 0.25);
    base_height_waypoint = this->declare_parameter<double>("standing_up.base_height_waypoint", 0.1);

    trajectory_freq_hz = this->declare_parameter<double>("trajectory.frequency_hz", 20);


    this->init_service_ = this->create_service<std_srvs::srv::Trigger>("/motion_manager_init", std::bind(&ElkapodMotionManager::initServiceCallback, this, _1, _2), 10);
    this->standup_service_ = this->create_service<std_srvs::srv::Trigger>("/motion_manager_standup", std::bind(&ElkapodMotionManager::standUpServiceCallback, this, _1, _2), 10);
    this->lower_service_ = this->create_service<std_srvs::srv::Trigger>("/motion_manager_lower", std::bind(&ElkapodMotionManager::lowerDownServiceCallback, this, _1, _2), 10);


    this->leg_positions_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/elkapod_leg_positions", 10);
    this->timer_ = this->create_wall_timer(50ms, std::bind(&ElkapodMotionManager::legControlCallback, this));
    this->state_ = State::INIT;
    
    this->planner = LinearLegPlanner();
    this->hop_planner = HopLegPlanner();
    this->executor = TrajectoryExecutor();
    this->executor_enable = false;
}

void ElkapodMotionManager::initNode(){
    RCLCPP_INFO(this->get_logger(), "Elkapod motion manager initialized!");
}

void ElkapodMotionManager::initPlanning(){
    const double max_reach_x = 0.38;
    const double movement_time_s = 10;

    std::array<Trajectory, 6> step_trajs;
    for(int i = 0; i < 6; ++i)  {
      auto traj = hop_planner.plan({max_reach_x, 0.0, 0.0}, {leg_spacing_waypoint, 0.0, 0.0}, movement_time_s, trajectory_freq_hz);
      step_trajs[i] = traj;
    }
    trajs.push_back(step_trajs);
}


void ElkapodMotionManager::standUpPlanning(){
    std::array<Trajectory, 6> step_trajs;
    // Second step - lift up a little bit
    for(int i = 0; i < 6; ++i)  {
      auto traj = planner.plan({leg_spacing_waypoint, 0.0, 0.0}, {leg_spacing_waypoint, 0.0, -base_height_waypoint}, 10, trajectory_freq_hz);
      step_trajs[i] = traj;
    }
    trajs.push_back(step_trajs);

    // Third step - hop each leg one at a time
    for(int i = 0; i < 6; ++i)  {
      for(int j = 0; j < 6; ++j){
        if(i == j){
            auto traj = hop_planner.plan({leg_spacing_waypoint, 0.0, -base_height_waypoint}, {leg_spacing, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
            step_trajs[j] = traj;
        }
        else if(j < i){
            auto traj = planner.plan({leg_spacing, 0.0, -base_height_waypoint}, {leg_spacing, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
            step_trajs[j] = traj;
        }
        else{
            auto traj = planner.plan({leg_spacing_waypoint, 0.0, -base_height_waypoint}, {leg_spacing_waypoint, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
            step_trajs[j] = traj;
        }
      }
      trajs.push_back(step_trajs);
    }

    // Final lift up
    for(int i = 0; i < 6; ++i)  {
      auto traj = planner.plan({leg_spacing, 0.0, -base_height_waypoint}, {0.2, 0.0, -base_height}, 10, trajectory_freq_hz);
      step_trajs[i] = traj;
    }
    trajs.push_back(step_trajs);
}

void ElkapodMotionManager::lowerDownPlanning(){
    std::array<Trajectory, 6> step_trajs;
    // Final lift up
    for(int i = 0; i < 6; ++i)  {
      auto traj = planner.plan({leg_spacing, 0.0, -base_height}, {leg_spacing, 0.0, -base_height_waypoint}, 10, trajectory_freq_hz);
      step_trajs[i] = traj;
    }
    trajs.push_back(step_trajs);

    // Third step - hop each leg one at a time
    for(int i = 0; i < 6; ++i)  {
      for(int j = 0; j < 6; ++j){
        if(i == j){
            auto traj = hop_planner.plan({leg_spacing, 0.0, -base_height_waypoint}, {leg_spacing_waypoint, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
            step_trajs[j] = traj;
        }
        else if(j < i){
            auto traj = planner.plan({leg_spacing_waypoint, 0.0, -base_height_waypoint}, {leg_spacing_waypoint, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
            step_trajs[j] = traj;
        }
        else{
            auto traj = planner.plan({leg_spacing, 0.0, -base_height_waypoint}, {leg_spacing, 0.0, -base_height_waypoint}, 2, trajectory_freq_hz);
            step_trajs[j] = traj;
        }
      }
      trajs.push_back(step_trajs);
    }

    // Second step - lift up a little bit
    for(int i = 0; i < 6; ++i)  {
      auto traj = planner.plan({leg_spacing_waypoint, 0.0, -base_height_waypoint}, {leg_spacing_waypoint, 0.0, 0.0}, 10, trajectory_freq_hz);
      step_trajs[i] = traj;
    }
    trajs.push_back(step_trajs);

}

void ElkapodMotionManager::initServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger_Request> request, std::shared_ptr<std_srvs::srv::Trigger_Response> response){ 
    initPlanning();

    auto current_traj = trajs.front();
    trajs.erase(trajs.begin());
    executor.setTrajectories(current_traj);
    executor_enable = true;
}

void ElkapodMotionManager::lowerDownServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger_Request> request, std::shared_ptr<std_srvs::srv::Trigger_Response> response){ 
    lowerDownPlanning();

    auto current_traj = trajs.front();
    trajs.erase(trajs.begin());
    executor.setTrajectories(current_traj);
    executor_enable = true;
}

void ElkapodMotionManager::standUpServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger_Request> request, std::shared_ptr<std_srvs::srv::Trigger_Response> response){ 
    standUpPlanning();

    auto current_traj = trajs.front();
    trajs.erase(trajs.begin());
    executor.setTrajectories(current_traj);
    executor_enable = true;
}

void ElkapodMotionManager::legControlCallback() {
    std_msgs::msg::Float64MultiArray output_msg;
    output_msg.data.resize(18);

    if(executor_enable && !executor.hasNext() && trajs.size() > 0){
        auto current_traj = trajs.front();
        trajs.erase(trajs.begin());
        executor.setTrajectories(current_traj);
        std::string msg2 = "Trajectories left: " + std::to_string(trajs.size());
        RCLCPP_INFO(this->get_logger(), msg2.c_str());

    }

    
    if(executor_enable && (executor.hasNext() || trajs.size() > 0) ){ 
      auto step = executor.next();
      size_t index = 0;
      for (const auto& leg : step) {
          for (double coordinate : leg) {
              if(trajs.size() == 0){
                std::string msg2 = "Coordinate " + std::to_string(coordinate);
              RCLCPP_INFO(this->get_logger(), msg2.c_str());
              }
              output_msg.data[index++] = coordinate;
              this->leg_positions_pub_->publish(output_msg);
          }
      }
    }
    else{
        executor_enable = false;
    }
}
