
//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#include <memory>
#include "elkapod_leg_inverse_kinematics.hpp"

typedef struct{
    std::vector<std::array<double, 3>> thetas;
    double timeStep;
} Trajectory;

class TrajectoryGenerator{
    public:
        TrajectoryGenerator(const std::shared_ptr<KinematicsSolver>& solver);
        const Trajectory generateLSPBTrajectory(const Eigen::Vector3d point, const double time);
    private:
        std::shared_ptr<KinematicsSolver> solver;
};

class TrajectoryExecutioner{
    public:
        TrajectoryExecutioner()
}