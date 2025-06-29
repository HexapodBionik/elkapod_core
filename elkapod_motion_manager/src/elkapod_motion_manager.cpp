#include "../include/elkapod_motion_manager/elkapod_motion_manager.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

ElkapodMotionManager::ElkapodMotionManager(): Node("elkapod_motion_manager"){
    this->init_service_ = this->create_service<std_srvs::srv::Trigger>("/motion_manager_init", std::bind(&ElkapodMotionManager::initServiceCallback, this, _1, _2), 10);

    this->leg_positions_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/elkapod_leg_positions", 10);
    this->timer_ = this->create_wall_timer(50ms, std::bind(&ElkapodMotionManager::legControlCallback, this));
    this->state_ = State::INIT;
    
    this->planner = LinearLegPlanner();
    this->executor = TrajectoryExecutor();
    this->executor_enable = false;
}

void ElkapodMotionManager::init(){
    leg_positions = {
                        {0.175, 0.0, 0.0},
                        {0.175, 0.0, 0.0},
                        {0.175, 0.0, 0.0},
                        {0.175, 0.0, 0.0},
                        {0.175, 0.0, 0.0},
                        {0.175, 0.0, 0.0},
                    };

    RCLCPP_INFO(this->get_logger(), "Initialized!");

}

void ElkapodMotionManager::initServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger_Request> request, std::shared_ptr<std_srvs::srv::Trigger_Response> response){
    for(int i = 0; i < 6; ++i)  {
      auto traj = planner.plan({0.25, 0.0, 0.0}, {0.25, 0.0, -0.1}, 10, 0.1);
      trajs[i] = traj;
    }
    executor.setTrajectories(trajs);
    executor_enable = true;
}

void ElkapodMotionManager::legControlCallback() {
    std_msgs::msg::Float64MultiArray output_msg;
    output_msg.data.resize(18);

    
    if(executor_enable && executor.hasNext()){
      auto step = executor.next();
      RCLCPP_INFO(this->get_logger(), "Publishing next trajectory point!");

      size_t index = 0;
      for (const auto& leg : step) {
          for (double coordinate : leg) {
              std::string msg2 = "Coordinate " + std::to_string(coordinate);
              RCLCPP_INFO(this->get_logger(), msg2.c_str());
              output_msg.data[index++] = coordinate;
              this->leg_positions_pub_->publish(output_msg);
          }
      }
      
    }
    else{
      executor_enable = false;
    }


    // size_t index = 0;
    // for (const auto& leg : leg_positions) {
    //     for (double coordinate : leg) {
    //         output_msg.data[index++] = coordinate;
    //     }
    // }

    //this->leg_positions_pub_->publish(output_msg);
}






