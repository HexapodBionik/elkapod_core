

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
        void initNode();
        
    private:
        void lowerDownPlanning();
        void standUpPlanning();
        void initPlanning();

        void standUpServiceCallback(std::shared_ptr<std_srvs::srv::Trigger_Request> request, std::shared_ptr<std_srvs::srv::Trigger_Response> response);
        void lowerDownServiceCallback(std::shared_ptr<std_srvs::srv::Trigger_Request> request, std::shared_ptr<std_srvs::srv::Trigger_Response> response);
        void initServiceCallback(std::shared_ptr<std_srvs::srv::Trigger_Request> request, std::shared_ptr<std_srvs::srv::Trigger_Response> response);
        void legControlCallback();

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr standup_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lower_service_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_positions_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        State state_;
        
        std::vector<std::array<Trajectory, 6>> trajs;
        LinearLegPlanner planner;
        HopLegPlanner hop_planner;
        TrajectoryExecutor executor;
        bool executor_enable;

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


#endif //ELKAPOD_MOTION_MANAGER_HPP
