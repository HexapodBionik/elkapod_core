#include "elkapod_gait_controller/leg_path.hpp"

#include <math.h>

using namespace elkapod_leg_paths;

void BasicPath::init() {
  x_func_stance_ = [this](double s) { return step_length_ / 2.0 - step_length_ * s; };
  y_func_stance_ = []([[maybe_unused]] double s) { return 0.0; };
  z_func_stance_ = []([[maybe_unused]] double s) { return 0.0; };

  x_func_swing_ = [this](double s) { return -step_length_ / 2.0 + step_length_ * s; };
  y_func_swing_ = []([[maybe_unused]] double s) { return 0.0; };

  z_func_swing_ = [this](double s) {
    double a = 0.0;

    if (step_length_ > 0) {
      a = -(step_height_) / pow(step_length_ / 2, 2);
    }

    const double x = -step_length_ / 2.0 + step_length_ * s;
    return a * (x - step_length_ / 2) * (x + step_length_ / 2);
  };

  initialized_ = true;
}

std::optional<UpdateParametersError> BasicPath::updateBasicParameters(double step_length,
                                                                      double step_height) noexcept {
  if (step_length < 0 || step_height < 0) {
    return UpdateParametersError::NEGATIVE_STEP_LENGTH_HEIGHT;
  }
  step_length_ = step_length;
  step_height_ = step_height;
  return {};
}

std::optional<Eigen::Vector3d> BasicPath::eval(double s, double phase) const noexcept {
  if (!initialized_) {
    return {};
  }

  if (phase) {
    return Eigen::Vector3d{x_func_swing_(s), y_func_swing_(s), z_func_swing_(s)};
  } else {
    return Eigen::Vector3d{x_func_stance_(s), y_func_stance_(s), z_func_stance_(s)};
  }
}

std::optional<Eigen::Vector3d> BasicPath::operator()(double s, double phase) const noexcept {
  return eval(s, phase);
}

void BasicPathBezier::init() {
  bezier_ = [](double s) { return 10 * std::pow(s, 3) - 15 * std::pow(s, 4) + 6 * std::pow(s, 5); };

  x_func_stance_ = [this](double s) { return step_length_ / 2.0 - step_length_ * bezier_(s); };
  y_func_stance_ = []([[maybe_unused]] double s) { return 0.0; };
  z_func_stance_ = []([[maybe_unused]] double s) { return 0.0; };

  x_func_swing_ = [this](double s) { return -step_length_ / 2.0 + step_length_ * bezier_(s); };
  y_func_swing_ = []([[maybe_unused]] double s) { return 0.0; };

  z_func_swing_ = [this](double s) {
    double a = -(step_height_) / pow(step_length_ / 2, 2);

    if (step_length_ <= 0) {
      a = 0.0;
    }
    const double x = -step_length_ / 2.0 + step_length_ * bezier_(s);
    return a * (x - step_length_ / 2) * (x + step_length_ / 2);
  };
  initialized_ = true;
}
