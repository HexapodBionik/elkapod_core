

//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#include "../include/elkapod_leg_control_tests/elkapod_leg_inverse_kinematics.hpp"
#include <math.h>
#include <iostream>

static inline double sign(double x){
    return (x > 0) - (x < 0);
}


static inline double rad2deg(double deg){
    return deg * M_PI / 180;
}


KinematicsSolver::KinematicsSolver(const std::vector<Eigen::Vector3d> linkLengths): m1_(linkLengths[0]),
    a1_(linkLengths[1]), a2_(linkLengths[2]), a3_(linkLengths[3]){

    validateConstructorArguments({linkLengths[0], linkLengths[1], linkLengths[2], linkLengths[3]});
}

void KinematicsSolver::validateConstructorArguments(const std::array<Eigen::Vector3d, 4>& values) {
    for (size_t i = 0; i < values.size(); ++i) {
        if (values[i].minCoeff() < 0) {
            throw std::invalid_argument("Value at index " + std::to_string(i) + " cannot be less than 0!");
        }
    }
}

Eigen::Vector3d KinematicsSolver::inverse(const Eigen::Vector3d& point){
    const double x = point[0];
    const double y = point[1];
    const double z = point[2] - m1_[2] - a1_[2];

    std::cout<<x<<"\t"<<y<<"\t"<<z<<std::endl;

    const double q1 = x > 0 ? atan2(y, x) : sign(y) * static_cast<double>(M_PI) / 2;

    const double span = (x > 0 ? sign(x) : 1) * sqrt(pow(x, 2) + pow(y, 2) - a1_[0]);

    const double P = sqrt(pow(span, 2) + pow(z, 2));

    double q2 = 0;

    if(P >= 0.3){
        if(a2_[0] != a3_[0]){
            throw std::runtime_error("Given point cannot be reached");
        }
    }
    else{
        const double argQ2 = (pow(P, 2) + pow(a2_[0], 2) - pow(a3_[0], 2)) / (2 * P * a2_[0]);

        q2 = ((z == 0 && span < 0) ? -static_cast<double>(M_PI) : atan2(z, span)) + acos(argQ2);
    }

    const double argQ3 = (pow(a2_[0], 2) + pow(a3_[0], 2) - pow(P, 2)) / (2 * a2_[0] * a3_[0]);
    const double q3 = acos(argQ3) - static_cast<double>(M_PI);

    Eigen::Vector3d degAngles = {rad2deg(q1), rad2deg(q2), rad2deg(q3)};
    return degAngles;
}