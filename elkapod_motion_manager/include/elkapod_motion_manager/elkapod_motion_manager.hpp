

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
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "elkapod_leg_trajectory.hpp"

using namespace std::chrono_literals;


typedef enum{
    INIT,
    IDLE_LOWERED,
    IDLE,
    WALKING
} State;

using namespace std::chrono_literals;

class ElkapodMotionManager: public rclcpp::Node
{
    public:
        ElkapodMotionManager();
        void init();
        
    private:
        void initServiceCallback(std::shared_ptr<std_srvs::srv::Trigger_Request> request, std::shared_ptr<std_srvs::srv::Trigger_Response> response);
        void legControlCallback();
        // void topicCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg);

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_service_;
        

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_positions_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        State state_;
        
        std::vector<std::vector<double>> leg_positions;
        // std::array<std::array<double, 3>, 6> leg_positions;
        // std::string config_path_;
        // double height;
        // double sign;
        
        std::array<Trajectory, 6> trajs;
        LinearLegPlanner planner;
        TrajectoryExecutor executor;
        bool executor_enable;

    
};


#endif //ELKAPOD_MOTION_MANAGER_HPP
