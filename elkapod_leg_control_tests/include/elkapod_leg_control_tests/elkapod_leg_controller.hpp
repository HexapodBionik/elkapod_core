

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
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "elkapod_leg_inverse_kinematics.hpp"


using namespace std::chrono_literals;

class ElkapodLegPublisher: public rclcpp::Node
{
    public:
        ElkapodLegPublisher();
        void init();
        
    private:
        void timerCallback();

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr my_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<KinematicsSolver> solver;
        std::string config_path_;
        double height;
        double sign;

    
};


#endif //ELKAPOD_LEG_CONTROLLER_HPP
