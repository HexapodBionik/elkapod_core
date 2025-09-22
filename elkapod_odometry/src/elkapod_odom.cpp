#include "../include/elkapod_odometry/elkapod_odom.hpp"
#include <format>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

ElkapodOdom::ElkapodOdom(): Node("elkapod_odom"){

    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&ElkapodOdom::jointStatesCallback, this, std::placeholders::_1));
    collision_sub_ = this->create_subscription<std_msgs::msg::Int8MultiArray>("/legs/fsr", 10, std::bind(&ElkapodOdom::collisionSubCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/leg_odom", 10);

    timer_ = this->create_timer(50ms, std::bind(&ElkapodOdom::odomCallback, this));

    odom_pose_ = Eigen::Matrix4d::Identity();
    contact_cache_ = {0.};
    leg_angles_ = {0.};
    last_leg_positions_ = std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero());

    base_link_rotations_ = {0.63973287, -0.63973287, M_PI / 2., -M_PI / 2., 2.38414364, -2.38414364};
    base_link_translations_ = {{0.17841, 0.13276, -0.03},  {0.17841, -0.13276, -0.03},
                             {0.0138, 0.1643, -0.03},    {0.0138, -0.1643, -0.03},
                             {-0.15903, 0.15038, -0.03}, {-0.15903, -0.15038, -0.03}};
    
    Eigen::Vector3d m1(0.0, 0.0, 0.025);
    Eigen::Vector3d a1(0.0676, 0.0, 0.0);
    Eigen::Vector3d a2(0.09237, 0.0, 0.0);
    Eigen::Vector3d a3(0.22524, 0.0, 0.0);

    const std::vector<Eigen::Vector3d> input = {m1, a1, a2, a3};
    solver_ = std::make_unique<KinematicsSolver>(input);
}

void ElkapodOdom::collisionSubCallback(const std_msgs::msg::Int8MultiArray::SharedPtr contacts){
    std::copy(contacts->data.begin(), contacts->data.end(), contact_cache_.begin());
}

void ElkapodOdom::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr joint_states){
    int j = 0;
    for(size_t i = 0; i < 24; ++i){
        if(i % 4 != 0){
            leg_angles_[j] = joint_states->position[i];
            ++j;
        }
    }
    joint_states_initialized_ = true;
}

void ElkapodOdom::odomCallback(){
    if(!joint_states_initialized_){
        return;
    }

    std::vector<Eigen::Vector3d> current_leg_positions(6, Eigen::Vector3d::Zero());

    for(size_t i = 0; i < 6; ++i){
        if(contact_cache_[i] == 1){
            Eigen::Vector3d angles = {leg_angles_[i*3], leg_angles_[i*3+1], leg_angles_[i*3+2]};
            auto point = Eigen::AngleAxis(base_link_rotations_[i], Eigen::Vector3d::UnitZ()).toRotationMatrix() * solver_->forward(angles) + base_link_translations_[i];
            current_leg_positions[i] = point;
            // RCLCPP_INFO(get_logger(), std::format("X: {:.2f}\tY: {:.2f}\tZ: {:.2f}", point[0], point[1], point[2]).c_str());
        }
    }

    if(position_initialized_){
        // Część wspólna zbiorów C(t) i C(t+1)
        std::vector<int> common_legs;
        for(size_t i = 0; i < 6; ++i){
            if(!last_leg_positions_[i].isZero() && !current_leg_positions[i].isZero()){
                common_legs.push_back(i);
            }
        }
        
        const size_t N = common_legs.size();
        // RCLCPP_INFO(get_logger(), std::format("N: {:}", N).c_str());
        if(N < 3){
            RCLCPP_ERROR(get_logger(), "CONTACT LOST!!!");
        }
        if(N >= 3){
            Eigen::Matrix3Xd P(3, N);
            Eigen::Matrix3Xd Q(3, N);
            int i = 0;
            for (auto leg_nb: common_legs) {
                P.col(i) = last_leg_positions_[leg_nb]; 
                Q.col(i) = current_leg_positions[leg_nb]; 
                ++i;
            }

            auto T = Eigen::umeyama(P, Q, false);
            odom_pose_ = odom_pose_ * T.inverse();
            double yaw = odom_pose_.block<3,3>(0, 0).eulerAngles(0, 1, 2)[2];
            RCLCPP_INFO(get_logger(), std::format("X: {:.2f} Y: {:.2f} Z: {:.2f} Yaw: {:.2f}", odom_pose_(0, 3), odom_pose_(1, 3), odom_pose_(2, 3), yaw).c_str());
        }
    }
    position_initialized_ = true;
    last_leg_positions_ = current_leg_positions;

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.frame_id = "leg_odom";
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.child_frame_id = "base_link";
    

    auto pose = geometry_msgs::msg::PoseWithCovariance();
    auto position = geometry_msgs::msg::Point();

    const Eigen::Quaterniond q(odom_pose_.block<3,3>(0, 0));
    q.normalized();

    geometry_msgs::msg::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    position.x = odom_pose_(0, 3);
    position.y = odom_pose_(1, 3);
    position.z = odom_pose_(2, 3);

    pose.pose.position = position;
    pose.pose.orientation = orientation;

    odom_msg.pose = pose;
    odom_pub_->publish(odom_msg);
}
