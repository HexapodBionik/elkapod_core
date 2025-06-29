#include "../include/elkapod_motion_manager/elkapod_leg_trajectory.hpp"


Trajectory::Trajectory(const double timestep){
    this->timestep = timestep;
}

void Trajectory::addPoint(const Vec3& p){
    points.push_back(p);
}


Vec3 Trajectory::at(size_t i) const{
    return points[i];
}

Vec3 Trajectory::interpolate(double t) const{
    size_t index = t / timestep;
    return at(index);
}

size_t Trajectory::size() const{
    return points.size();
}

double Trajectory::totalDuration() const{
    return points.size() * timestep;
}


Trajectory LinearLegPlanner::plan(const Vec3& start, const Vec3& goal, double duration, double dt){
    auto x_func = [start, goal](double s){return start[0] + (goal[0] - start[0])*s;};
    auto y_func = [start, goal](double s){return start[1] + (goal[1] - start[1])*s;};
    auto z_func = [start, goal](double s){return start[2] + (goal[2] - start[2])*s;};

    size_t steps = duration / dt;
    Trajectory traj(dt);

    for(size_t i = 0; i < steps; ++i){
        double s = static_cast<double>(i) / static_cast<double>(steps);
        Vec3 p = {x_func(s), y_func(s), z_func(s)};
        traj.addPoint(p);
    }
    return traj;
}

TrajectoryExecutor::TrajectoryExecutor(){
    this->current_step = 0;
    this->max_steps = 0;
}

void TrajectoryExecutor::setTrajectories(const std::array<Trajectory, 6>& trajs) {
    trajectories = trajs;
    current_step = 0;
    max_steps = 0;
    for (const auto& t : trajs) {
        if (t.size() > max_steps) max_steps = t.size();
    }
}

bool TrajectoryExecutor::hasNext() const {
    return current_step < max_steps;
}

std::array<Vec3, 6> TrajectoryExecutor::next() {
    std::array<Vec3, 6> out;
    for (int i = 0; i < 6; ++i) {
        if (current_step < trajectories[i].size()) {
            out[i] = trajectories[i].at(current_step);
        } else {
            out[i] = trajectories[i].at(trajectories[i].size() - 1); 
        }
    }
    ++current_step;
    return out;
}