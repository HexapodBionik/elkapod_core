#include "../include/elkapod_odometry/elkapod_odom.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <format>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

ElkapodOdom::ElkapodOdom() : Node("elkapod_odom") {
  this->declare_parameter<double>("kinematics_solver.m1.m1_0");
  this->declare_parameter<double>("kinematics_solver.m1.m1_1");
  this->declare_parameter<double>("kinematics_solver.m1.m1_2");

  this->declare_parameter<double>("kinematics_solver.a1.a1_0");
  this->declare_parameter<double>("kinematics_solver.a1.a1_1");
  this->declare_parameter<double>("kinematics_solver.a1.a1_2");

  this->declare_parameter<double>("kinematics_solver.a2.a2_0");
  this->declare_parameter<double>("kinematics_solver.a2.a2_1");
  this->declare_parameter<double>("kinematics_solver.a2.a2_2");

  this->declare_parameter<double>("kinematics_solver.a3.a3_0");
  this->declare_parameter<double>("kinematics_solver.a3.a3_1");
  this->declare_parameter<double>("kinematics_solver.a3.a3_2");

  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ElkapodOdom::jointStatesCallback, this, std::placeholders::_1));
  collision_sub_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
      "/legs/fsr", 10, std::bind(&ElkapodOdom::collisionSubCallback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/leg_odom", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  timer_ = this->create_timer(12500us, std::bind(&ElkapodOdom::odomCallback, this));
  broadcaster_timer_ = this->create_timer(10ms, std::bind(&ElkapodOdom::tfCallback, this));

  odom_pose_ = Eigen::Matrix4d::Identity();
  previous_odom_pose_ = Eigen::Matrix4d::Identity();
  base_footprint_ = Eigen::Vector3d::Zero();
  contact_cache_ = {0.};
  leg_angles_ = {0.};
  last_leg_positions_ = std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero());

  current_leg_config_ = std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero());
  previous_leg_config_ = std::vector<Eigen::Vector3d>(6, Eigen::Vector3d::Zero());

  base_link_rotations_ = {0.63973287, -0.63973287, M_PI / 2., -M_PI / 2., 2.38414364, -2.38414364};
  base_link_translations_ = {{0.17841, 0.13276, -0.03},  {0.17841, -0.13276, -0.03},
                             {0.0138, 0.1643, -0.03},    {0.0138, -0.1643, -0.03},
                             {-0.15903, 0.15038, -0.03}, {-0.15903, -0.15038, -0.03}};

  last_odom_estimate_time_ = get_clock()->now().seconds();
  const Eigen::Vector3d m1(get_parameter("kinematics_solver.m1.m1_0").as_double(),
                           get_parameter("kinematics_solver.m1.m1_1").as_double(),
                           get_parameter("kinematics_solver.m1.m1_2").as_double());

  const Eigen::Vector3d a1(get_parameter("kinematics_solver.a1.a1_0").as_double(),
                           get_parameter("kinematics_solver.a1.a1_1").as_double(),
                           get_parameter("kinematics_solver.a1.a1_2").as_double());

  const Eigen::Vector3d a2(get_parameter("kinematics_solver.a2.a2_0").as_double(),
                           get_parameter("kinematics_solver.a2.a2_1").as_double(),
                           get_parameter("kinematics_solver.a2.a2_2").as_double());

  const Eigen::Vector3d a3(get_parameter("kinematics_solver.a3.a3_0").as_double(),
                           get_parameter("kinematics_solver.a3.a3_1").as_double(),
                           get_parameter("kinematics_solver.a3.a3_2").as_double());

  const std::vector<Eigen::Vector3d> input = {m1, a1, a2, a3};
  solver_ = std::make_shared<KinematicsSolver>(input);
}

void ElkapodOdom::collisionSubCallback(const std_msgs::msg::Int8MultiArray::SharedPtr contacts) {
  std::copy(contacts->data.begin(), contacts->data.end(), contact_cache_.begin());
}

void ElkapodOdom::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr joint_states) {
  int j = 0;
  for (size_t i = 0; i < 24; ++i) {
    if (i % 4 != 0) {
      leg_angles_[j] = joint_states->position[i];
      ++j;
    }
  }
  joint_states_initialized_ = true;
}

Eigen::Vector4d ElkapodOdom::findPlane(const Eigen::Matrix3Xd contact_points) {
  Eigen::Matrix<double, Eigen::Dynamic, 4> A(contact_points.cols(), 4);
  Eigen::VectorXd b(contact_points.cols());
  b.setZero();

  for (int i = 0; i < contact_points.cols(); ++i) {
    auto point = contact_points.col(i);
    A(i, 0) = point[0];
    A(i, 1) = point[1];
    A(i, 2) = point[2];
    A(i, 3) = -1.0;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  Eigen::Vector4d plane = svd.matrixV().col(3);
  return plane;
}

Eigen::Vector3d ElkapodOdom::findBaseFootprintCoords(Eigen::Vector4d plane) {
  Eigen::Vector3d v;
  v[0] = plane[0];
  v[1] = plane[1];
  v[2] = plane[2];
  const double d = plane[3];

  auto p = v * d / v.squaredNorm();
  // RCLCPP_INFO(get_logger(), std::format("X: {:.2f}\tY: {:.2f}\tZ: {:.2f}", p[0],
  //    p[1], p[2]).c_str());

  return p;
}

void ElkapodOdom::tfCallback() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "base_footprint";
  t.child_frame_id = "base_link";

  t.transform.translation.x = base_footprint_[0];
  t.transform.translation.y = base_footprint_[1];
  t.transform.translation.z = -base_footprint_[2];

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(t);
}

nav_msgs::msg::Odometry ElkapodOdom::fillOdomMsg(const Eigen::Matrix4d odom_pose,
                                                 const Eigen::Matrix4d /*previous_odom_pose*/) {
  auto odom_msg = nav_msgs::msg::Odometry();
  auto time = get_clock()->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.header.stamp = time;
  odom_msg.child_frame_id = "base_footprint";

  auto pose = geometry_msgs::msg::PoseWithCovariance();
  auto position = geometry_msgs::msg::Point();

  const Eigen::Quaterniond q(odom_pose.block<3, 3>(0, 0));
  q.normalized();

  geometry_msgs::msg::Quaternion orientation;
  orientation.x = q.x();
  orientation.y = q.y();
  orientation.z = q.z();
  orientation.w = q.w();

  position.x = odom_pose(0, 3);
  position.y = odom_pose(1, 3);
  position.z = odom_pose(2, 3);

  pose.pose.position = position;
  pose.pose.orientation = orientation;

  // 6x6 covariance matrix
  pose.covariance = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  odom_msg.pose = pose;

  double dt = time.seconds() - last_odom_estimate_time_;
  if (dt <= 1e-6) dt = 1e-6;
  auto twist = geometry_msgs::msg::TwistWithCovariance();

  Eigen::Vector3d p(odom_pose(0, 3), odom_pose(1, 3), odom_pose(2, 3));
  Eigen::Vector3d prev_p(previous_odom_pose_(0, 3), previous_odom_pose_(1, 3),
                         previous_odom_pose_(2, 3));
  Eigen::Vector3d v_world = (p - prev_p) / dt;
  Eigen::Matrix3d R_world_body = odom_pose.block<3, 3>(0, 0);
  Eigen::Vector3d v_body = R_world_body.transpose() * v_world;

  twist.twist.linear.x = v_body.x();
  twist.twist.linear.y = v_body.y();
  twist.twist.linear.z = v_body.z();

  twist.covariance = {
      0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  };

  odom_msg.twist = twist;
  last_odom_estimate_time_ = time.seconds();
  return odom_msg;
}

void ElkapodOdom::odomCallback() {
  if (!joint_states_initialized_) {
    return;
  }

  std::vector<Eigen::Vector3d> current_leg_positions(6, Eigen::Vector3d::Zero());

  for (size_t i = 0; i < 6; ++i) {
    if (contact_cache_[i] == 1) {
      Eigen::Vector3d angles = {leg_angles_[i * 3], leg_angles_[i * 3 + 1], leg_angles_[i * 3 + 2]};
      current_leg_config_[i] = angles;
      auto point =
          Eigen::AngleAxis(base_link_rotations_[i], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
              solver_->forward(angles) +
          base_link_translations_[i];
      current_leg_positions[i] = point;
    }
  }

  if (position_initialized_) {
    // Common part of sets C(t) and C(t+1)
    std::vector<int> common_legs;
    for (size_t i = 0; i < 6; ++i) {
      if (!last_leg_positions_[i].isZero() && !current_leg_positions[i].isZero()) {
        common_legs.push_back(i);
      }
    }

    const size_t N = common_legs.size();
    if (N < 3) {
      RCLCPP_ERROR(get_logger(), "CONTACT LOST!!!");
    }
    if (N >= 3) {
      Eigen::Matrix3Xd P(3, N);
      Eigen::Matrix3Xd Q(3, N);
      int i = 0;
      for (auto leg_nb : common_legs) {
        Q.col(i) = last_leg_positions_[leg_nb];
        P.col(i) = current_leg_positions[leg_nb];
        ++i;
      }
      auto T = Eigen::umeyama(P, Q, false);
      previous_odom_pose_ = odom_pose_;
      odom_pose_ = odom_pose_ * T;

      auto plane = findPlane(Q);
      base_footprint_ = findBaseFootprintCoords(plane);

      auto odom_msg = fillOdomMsg(odom_pose_, previous_odom_pose_);
      odom_pub_->publish(odom_msg);
    }
  }
  position_initialized_ = true;
  last_leg_positions_ = current_leg_positions;
  previous_leg_config_ = current_leg_config_;
}
