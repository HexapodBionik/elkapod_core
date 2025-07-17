

//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#include <eigen3/Eigen/Eigen>
#include <vector>

class KinematicsSolver {
 public:
  KinematicsSolver(const std::vector<Eigen::Vector3d> linkLengths);
  Eigen::Vector3d inverse(const Eigen::Vector3d& point);

 private:
  void validateConstructorArguments(const std::array<Eigen::Vector3d, 4>& arguments);
  const Eigen::Vector3d m1_;
  const Eigen::Vector3d a1_;
  const Eigen::Vector3d a2_;
  const Eigen::Vector3d a3_;
};