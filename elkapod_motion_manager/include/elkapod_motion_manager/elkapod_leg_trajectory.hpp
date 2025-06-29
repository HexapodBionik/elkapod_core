

//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef ELKAPOD_LEG_TRAJECTORY_HPP
#define ELKAPOD_LEG_TRAJECTORY_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Eigen>

using Vec3 = Eigen::Vector3d;

class Trajectory {
    std::vector<Vec3> points;
    double timestep;

public:
    Trajectory() : timestep(0.0) {}
    Trajectory(double timestep);
    void addPoint(const Vec3& p);
    Vec3 at(size_t i) const;
    Vec3 interpolate(double t) const;
    size_t size() const;
    double totalDuration() const;
};

class LegPlanner{
    public:
        virtual ~LegPlanner() = default;
        virtual Trajectory plan(const Vec3& start, const Vec3& goal, double duration, double dt) = 0;
    private:
};

class LinearLegPlanner: public LegPlanner{
    public:
        LinearLegPlanner() = default;
        Trajectory plan(const Vec3& start, const Vec3& goal, double duration, double dt) override;
};

class TrajectoryExecutor{
    public:
        TrajectoryExecutor();
        void setTrajectories(const std::array<Trajectory, 6>& trajs);
        bool hasNext() const;
        std::array<Vec3, 6> next();
    private:
        size_t current_step;
        size_t max_steps;
        std::array<Trajectory, 6> trajectories;
};

#endif //ELKAPOD_LEG_TRAJECTORY_HPP
