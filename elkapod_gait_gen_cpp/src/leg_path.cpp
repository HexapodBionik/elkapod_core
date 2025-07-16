#include "../include/elkapod_gait_gen_cpp/leg_path.hpp"

#include <math.h>

#include <stdexcept>

void ElkapodLegPath::init() {
  const double R =
      (std::pow(step_height_, 2) + std::pow(step_length_, 2) / 4.) / (2 * step_height_);
  const double h = R - step_height_;
  if (h < 0) {
    throw std::invalid_argument("Invalid step parameters: computed height offset is negative.");
  }

  x_func_stance_ = [this](double s) { return step_length_ / 2.0 - step_length_ * s; };
  y_func_stance_ = [](double s) { return 0.0; };
  z_func_stance_ = [](double s) { return 0.0; };

  x_func_swing_ = [this](double s) { return -step_length_ / 2.0 + step_length_ * s; };
  y_func_swing_ = [](double s) { return 0.0; };
  z_func_swing_ = [R](double s) { return R * std::sin(M_PI * s); };
}

Eigen::Vector3d ElkapodLegPath::operator()(double s, double phase) const {
  if (phase) {
    if (!x_func_swing_ || !y_func_swing_ || !z_func_swing_)
      throw std::runtime_error("Swing functions are not initialized.");
    return {x_func_swing_(s), y_func_swing_(s), z_func_swing_(s)};
  } else {
    if (!x_func_stance_ || !y_func_stance_ || !z_func_stance_)
      throw std::runtime_error("Stance functions are not initialized.");
    return {x_func_stance_(s), y_func_stance_(s), z_func_stance_(s)};
  }
}
