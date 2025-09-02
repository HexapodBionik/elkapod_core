//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#ifndef LEG_PATH_HPP
#define LEG_PATH_HPP

#include <eigen3/Eigen/Eigen>
#include <functional>
#include <optional>

namespace elkapod_leg_paths {
enum class UpdateParametersError { NEGATIVE_STEP_LENGTH_HEIGHT = 1 };

class BasicPath {
 public:
  BasicPath(const double step_length, const double step_height)
      : step_length_(step_length), step_height_(step_height){};

  virtual ~BasicPath(){};

  virtual void init();
  virtual std::optional<UpdateParametersError> updateBasicParameters(double step_length,
                                                                     double step_height) noexcept;
  virtual std::optional<Eigen::Vector3d> eval(double s, double phase) const noexcept;
  std::optional<Eigen::Vector3d> operator()(double s, double phase) const noexcept;

 protected:
  double step_length_;
  double step_height_;
  bool initialized_ = false;

  std::function<double(double)> x_func_stance_ = nullptr;
  std::function<double(double)> y_func_stance_ = nullptr;
  std::function<double(double)> z_func_stance_ = nullptr;

  std::function<double(double)> x_func_swing_ = nullptr;
  std::function<double(double)> y_func_swing_ = nullptr;
  std::function<double(double)> z_func_swing_ = nullptr;
};

class BasicPathBezier : public BasicPath {
 public:
  BasicPathBezier(const double step_length, const double step_height)
      : BasicPath(step_length, step_height){};

  void init() override;

 private:
  std::function<double(double)> bezier_ = nullptr;
};
};  // namespace elkapod_leg_paths

#endif  // LEG_PATH_HPP
