#include "elkapod_core_lib/control.hpp"

#include <algorithm>

using namespace elkapod_core_lib::control;

PID::PID(const double K, const double Ti, const double Td, const double T) {
  updateCoefficients(K, Ti, Td, T);
  e_[0] = 0.0;
  e_[1] = 0.0;
  e_[2] = 0.0;
  ukm1_ = 0.0;
}

void PID::setCommandLimits(const double lo, const double hi) {
  command_lo_limit_ = lo;
  command_hi_limit_ = hi;
}

void PID::updateCoefficients(const double K, const double Ti, const double Td, const double T) {
  T_ = T;
  r2_ = K * Td / T;
  r1_ = K * (T / (2 * Ti) - 2 * Td / T - 1);
  r0_ = K * (1 + T / (2 * Ti) + Td / T);
}

double PID::update(const double e) {
  e_[2] = e_[1];
  e_[1] = e_[0];
  e_[0] = e;

  double u = r2_ * e_[2] + r1_ * e_[1] + r0_ * e_[0] + ukm1_;
  u = std::clamp(u, command_lo_limit_, command_hi_limit_);
  ukm1_ = u;

  return u;
}