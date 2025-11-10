//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#include "elkapod_core_lib/leg_kinematics.hpp"

#include <math.h>

#include <eigen3/Eigen/Geometry>
#include <iostream>

using namespace elkapod_core_lib::kinematics;

static inline double sign(double x) { return (x > 0) - (x < 0); }

KinematicsSolver::KinematicsSolver(const std::vector<Eigen::Vector3d> linkLengths)
    : m1_(linkLengths[0]), a1_(linkLengths[1]), a2_(linkLengths[2]), a3_(linkLengths[3]) {
  validateConstructorArguments({linkLengths[0], linkLengths[1], linkLengths[2], linkLengths[3]});
}

void KinematicsSolver::validateConstructorArguments(const std::array<Eigen::Vector3d, 4>& values) {
  for (size_t i = 0; i < values.size(); ++i) {
    if (values[i].minCoeff() < 0) {
      throw std::invalid_argument("Value at index " + std::to_string(i) +
                                  " cannot be less than 0!");
    }
  }
}

Eigen::Vector3d KinematicsSolver::inverse(const Eigen::Vector3d& point) const {
  const double x = point[0];
  const double y = point[1];
  const double z = point[2] - m1_[2] - a1_[2];

  const double q1 = x > 0 ? atan2(y, x) : sign(y) * static_cast<double>(M_PI) / 2;

  const double span = (x > 0 ? sign(x) : 1) * sqrt(pow(x, 2) + pow(y, 2)) - a1_[0];

  const double P = sqrt(pow(span, 2) + pow(z, 2));

  double q2 = 0;

  if (P >= 0.39) {
    if (a2_[0] != a3_[0]) {
      throw std::runtime_error("Given point cannot be reached");
    }
  } else {
    const double argQ2 = (pow(P, 2) + pow(a2_[0], 2) - pow(a3_[0], 2)) / (2 * P * a2_[0]);

    q2 = ((z == 0 && span < 0) ? -static_cast<double>(M_PI) : atan2(z, span)) + acos(argQ2);
  }

  const double argQ3 = (pow(a2_[0], 2) + pow(a3_[0], 2) - pow(P, 2)) / (2 * a2_[0] * a3_[0]);
  const double q3 = acos(argQ3) - static_cast<double>(M_PI);

  return {q1, q2, q3};
}

Eigen::Vector3d KinematicsSolver::forward(const Eigen::Vector3d& angles) const noexcept {
  Eigen::Matrix3d rot_0_1 =
      Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix3d rot_1_2 = (Eigen::AngleAxisd(0.5 * EIGEN_PI, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitZ()))
                                .toRotationMatrix();
  Eigen::Matrix3d rot_2_3 =
      Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();

  Eigen::Vector3d trans_0_1 = {0., 0., m1_[2]};
  Eigen::Vector3d trans_1_2 = a1_;
  Eigen::Vector3d trans_2_3 = a2_;
  Eigen::Vector3d trans_3_4 = a3_;

  Eigen::Matrix4d t_0_1, t_1_2, t_2_3, t_3_4;

  t_0_1.setIdentity();
  t_0_1.block<3, 3>(0, 0) = rot_0_1;
  t_0_1.block<3, 1>(0, 3) = trans_0_1;

  t_1_2.setIdentity();
  t_1_2.block<3, 3>(0, 0) = rot_1_2;
  t_1_2.block<3, 1>(0, 3) = trans_1_2;

  t_2_3.setIdentity();
  t_2_3.block<3, 3>(0, 0) = rot_2_3;
  t_2_3.block<3, 1>(0, 3) = trans_2_3;

  t_3_4.setIdentity();
  t_3_4.block<3, 1>(0, 3) = trans_3_4;

  auto t_0_4 = t_0_1 * t_1_2 * t_2_3 * t_3_4;
  return t_0_4.block<3, 1>(0, 3);
}