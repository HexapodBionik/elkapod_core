

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

class ElkapodLegPath {
 public:
  ElkapodLegPath(const double step_length, const double step_height)
      : step_length_(step_length), step_height_(step_height){};
  void init();
  Eigen::Vector3d operator()(double s, double phase) const;

 private:
  const double step_length_;
  const double step_height_;

  std::function<double(double)> x_func_stance_;
  std::function<double(double)> y_func_stance_;
  std::function<double(double)> z_func_stance_;

  std::function<double(double)> x_func_swing_;
  std::function<double(double)> y_func_swing_;
  std::function<double(double)> z_func_swing_;
};

#endif  // LEG_PATH_HPP
