//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#include "../include/elkapod_leg_control_tests/elkapod_trajectory_executioner.hpp"


TrajectoryGenerator::TrajectoryGenerator(const std::shared_ptr<KinematicsSolver>& solver){
    this->solver = solver;
}

const Trajectory TrajectoryGenerator::generateLSPBTrajectory(const Eigen::Vector3d point, const double time){

}


