

//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//


#ifndef ELKAPOD_LEG_KINEMATICS
#define ELKAPOD_LEG_KINEMATICS

#include <eigen3/Eigen/Eigen>
#include <vector>

class KinematicsSolver {
 public:
  KinematicsSolver()=default;
  KinematicsSolver(const std::vector<Eigen::Vector3d> linkLengths);
  Eigen::Vector3d inverse(const Eigen::Vector3d& point) const;
  Eigen::Vector3d forward(const Eigen::Vector3d& angles) const noexcept;

 private:
  void validateConstructorArguments(const std::array<Eigen::Vector3d, 4>& arguments);
  const Eigen::Vector3d m1_;
  const Eigen::Vector3d a1_;
  const Eigen::Vector3d a2_;
  const Eigen::Vector3d a3_;
};

#endif // ELKAPOD_LEG_KINEMATICS