//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#include "../include/elkapod_leg_control_tests/elkapod_leg_controller.hpp"
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>

using std::placeholders::_1;

static inline float deg2rad(float deg){
    return deg / 180.f * M_PI;
}


ElkapodLegPublisher::ElkapodLegPublisher(): Node("elkapod_leg_publisher"){
    this->declare_parameter("config_path", "");

    this->get_parameter("config_path", config_path_);

    if(config_path_.empty()){
        throw std::invalid_argument("Config path cannot be empty!");
    }

    this->my_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_position_controller/commands", 10);
    this->my_subscription_ =  this->create_subscription<std_msgs::msg::Float64MultiArray>("elkapod_leg_positions", 10, std::bind(&ElkapodLegPublisher::topicCallback, this, _1));
    //this->timer_ = this->create_wall_timer(20ms, std::bind(&ElkapodLegPublisher::timerCallback, this));
    this->height = -0.1;
    this->sign = 1;
}

void ElkapodLegPublisher::init(){
    YAML::Node config = YAML::LoadFile(config_path_);

    YAML::Node diagram_variables = config["movement_core"]["diagram_variables"];

    auto m1_v = diagram_variables["m1"].as<std::vector<double>>();
    auto a1_v = diagram_variables["a1"].as<std::vector<double>>();
    auto a2_v = diagram_variables["a2"].as<std::vector<double>>();
    auto a3_v = diagram_variables["a3"].as<std::vector<double>>();

    Eigen::Vector3d m1(m1_v[0], m1_v[1], m1_v[2]);
    Eigen::Vector3d a1(a1_v[0], a1_v[1], a1_v[2]);
    Eigen::Vector3d a2(a2_v[0], a2_v[1], a2_v[2]);
    Eigen::Vector3d a3(a3_v[0], a3_v[1], a3_v[2]);

    const std::vector<Eigen::Vector3d> input = {m1, a1, a2, a3};
    this->solver = std::make_shared<KinematicsSolver>(input);
    RCLCPP_INFO(this->get_logger(), "Initialized!");
}

void ElkapodLegPublisher::topicCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg){
    /*  Topic input: 3x6=18 values in following order x1, y1, z1, x2, y2, z2, x3, ...
    */

    auto output_msg = std_msgs::msg::Float64MultiArray();
    output_msg.data.resize(18);
    Eigen::Vector3d input;
    for(int i = 0; i < 6; ++i){
        input[0] = msg->data[i*3];
        input[1] = msg->data[i*3 + 1];
        input[2] = msg->data[i*3 + 2];

        std::string msg = "Point for leg " + std::to_string(i+1) + " x: " + std::to_string(input[0]) + " y: " + std::to_string(input[1]) + " z: " + std::to_string(input[2]);
        RCLCPP_INFO(this->get_logger(), msg.c_str());

        Eigen::Vector3d anglesDeg = this->solver->inverse(input);
        output_msg.data[i*3] = deg2rad(anglesDeg[0]);
        output_msg.data[i*3 + 1] = deg2rad(anglesDeg[1]);
        output_msg.data[i*3 + 2] = deg2rad(anglesDeg[2]);

        std::string msg2 = "Angles for leg " + std::to_string(i+1) + " theta0: " + std::to_string(output_msg.data[i*3]) + " theta1: " + std::to_string(output_msg.data[i*3+1]) + " theta2: " + std::to_string(output_msg.data[i*3+2]);
        RCLCPP_INFO(this->get_logger(), msg2.c_str());
    }
    this->my_publisher_->publish(output_msg);
}



void ElkapodLegPublisher::timerCallback(){
    if(height > -0.2 && sign == 1){
        height -= 0.0005;
    }
    else if(height < -0.1 && sign == -1){
        height += 0.0005;
    }

    if(height <= -0.2 && sign == 1){
        sign = -1;
    }
    else if(height >= -0.1 && sign == -1){
        sign = 1;
    }

    std::string current_height_str = "Current height" + std::to_string(height);

    //RCLCPP_INFO(this->get_logger(), current_height_str.c_str());
    Eigen::Vector3d input(0.2, 0.0, height);
    Eigen::Vector3d anglesDeg = this->solver->inverse(input);

    auto msg =  std_msgs::msg::Float64MultiArray();
    std::vector<float> angles(18, 0);

    std::vector<float> pattern = {deg2rad(anglesDeg[0]), deg2rad(anglesDeg[1]), deg2rad(anglesDeg[2])};

    std::string current_angles = "Q1: " + std::to_string(pattern[0]) + "\t" + "Q2: " + std::to_string(pattern[1]) + "\t" + "Q3: " + std::to_string(pattern[2]);
    RCLCPP_INFO(this->get_logger(), current_angles.c_str());

    for(int i = 0; i < 18; ++i){
        angles[i] = pattern[i % 3];
    }

    std::copy(angles.cbegin(), angles.cend(), std::back_inserter(msg.data));


    this->my_publisher_->publish(msg);

}



