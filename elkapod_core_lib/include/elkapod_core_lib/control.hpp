//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <limits>

namespace elkapod_core_lib::control {
class PID {
 public:
  PID() = default;
  PID(double K, double Ti, double Td, double T);
  void updateCoefficients(double K, double Ti, double Td, double T);
  void setCommandLimits(double lo, double hi);
  double update(double e);

 private:
  double command_lo_limit_ = -std::numeric_limits<double>::infinity();
  double command_hi_limit_ = std::numeric_limits<double>::infinity();
  double T_;
  double r2_, r1_, r0_;
  double ukm1_;
  double e_[3];
};
};  // namespace elkapod_core_lib::control
#endif