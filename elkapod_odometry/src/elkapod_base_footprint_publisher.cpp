#include "../include/elkapod_odometry/elkapod_base_footprint_publisher.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cstdio>
#include <format>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

ElkapodBaseFootprintPublisher::ElkapodBaseFootprintPublisher() : Node("elkapod_base_footprint_publisher") {
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
      std::bind(&ElkapodBaseFootprintPublisher::jointStatesCallback, this, std::placeholders::_1));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  timer_ = this->create_timer(12500us, std::bind(&ElkapodBaseFootprintPublisher::computeBaseFootprintCallback, this));
  broadcaster_timer_ = this->create_timer(10ms, std::bind(&ElkapodBaseFootprintPublisher::tfCallback, this));

  base_footprint_ = Eigen::Vector3d::Zero();
  leg_angles_ = {0.};

  base_link_rotations_ = {0.63973287, -0.63973287, M_PI / 2., -M_PI / 2., 2.38414364, -2.38414364};
  base_link_translations_ = {{0.17841, 0.13276, -0.03},  {0.17841, -0.13276, -0.03},
                             {0.0138, 0.1643, -0.03},    {0.0138, -0.1643, -0.03},
                             {-0.15903, 0.15038, -0.03}, {-0.15903, -0.15038, -0.03}};

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

  joint_states_initialized_ = false;
  position_initialized_ = false;
}

void ElkapodBaseFootprintPublisher::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr joint_states) {
  // Map joints by name ("leg<N>_J<K>" -> leg_angles_[(N-1)*3 + (K-1)]), the message order is not fixed
  const size_t count = std::min(joint_states->name.size(), joint_states->position.size());
  std::array<bool, 18> received{};
  for (size_t i = 0; i < count; ++i) {
    int leg = 0, joint = 0;
    if (std::sscanf(joint_states->name[i].c_str(), "leg%d_J%d", &leg, &joint) != 2 || leg < 1 || leg > 6 ||
        joint < 1 || joint > 3) {
      continue;
    }
    const size_t idx = (leg - 1) * 3 + (joint - 1);
    leg_angles_[idx] = joint_states->position[i];
    received[idx] = true;
  }

  if (!joint_states_initialized_ &&
      std::all_of(received.begin(), received.end(), [](bool r) { return r; })) {
    joint_states_initialized_ = true;
  }
}

Eigen::Vector4d ElkapodBaseFootprintPublisher::findPlane(const Eigen::Matrix3Xd contact_points) {
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

Eigen::Vector3d ElkapodBaseFootprintPublisher::findBaseFootprintCoords(Eigen::Vector4d plane) {
  Eigen::Vector3d v;
  v[0] = plane[0];
  v[1] = plane[1];
  v[2] = plane[2];
  const double d = plane[3];

  auto p = v * d / v.squaredNorm();
  return p;
}

void ElkapodBaseFootprintPublisher::tfCallback() {
  auto now = this->get_clock()->now();
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "base_link";
  t.child_frame_id = "base_footprint";

  t.transform.translation.x = base_footprint_[0];
  t.transform.translation.y = base_footprint_[1];
  t.transform.translation.z = base_footprint_[2];

  t.transform.rotation.x = base_footprint_rotation_.x();
  t.transform.rotation.y = base_footprint_rotation_.y();
  t.transform.rotation.z = base_footprint_rotation_.z();
  t.transform.rotation.w = base_footprint_rotation_.w();

  tf_broadcaster_->sendTransform(t);
}


void ElkapodBaseFootprintPublisher::computeBaseFootprintCallback() {
  if (!joint_states_initialized_) {
    return;
  }

  Eigen::Matrix3Xd P(3, 6);
  int valid = 0;

  for (size_t i = 0; i < 6; ++i) {
    Eigen::Vector3d angles = {leg_angles_[i * 3], leg_angles_[i * 3 + 1], leg_angles_[i * 3 + 2]};
    Eigen::Matrix3d rot_matrix =
        Eigen::AngleAxis(base_link_rotations_[i], Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Vector3d fk_pos = solver_->forward(angles);

    if (fk_pos.hasNaN()) {
      continue;
    }
    P.col(valid++) = (rot_matrix * fk_pos) + base_link_translations_[i];
  }

  // A plane needs at least 3 points, keep the last estimate otherwise
  if (valid < 3) {
    return;
  }
  P.conservativeResize(Eigen::NoChange, valid);

  if (position_initialized_) {
    auto plane = findPlane(P);
    base_footprint_ = findBaseFootprintCoords(plane);

    Eigen::Vector3d normal = plane.head<3>().normalized();
    if (normal.dot(base_footprint_) > 0.0) {
      normal = -normal;
    }
    base_footprint_rotation_ = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal);
  }

  position_initialized_ = true;
}
