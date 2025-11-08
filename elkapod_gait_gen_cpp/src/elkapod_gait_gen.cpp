//
// Created by Piotr Patek.
//
// Copyright (c) 2025.
// Elkapod Bionik, Warsaw University of Technology. All rights reserved.
//

#include "../include/elkapod_gait_gen_cpp/elkapod_gait_gen.hpp"

#include <algorithm>
#include <format>
#include <functional>

namespace {
constexpr auto INPUT_VEL_TOL = 1e-5;
constexpr auto VEL_TOL = 1e-3;

constexpr auto EMA_FILTER_TAU = 0.15;
constexpr auto EMA_FILTER_TAU_BASE_HEIGHT = 0.15;

}  // namespace

static bool is_close(double a, double b, double atol = 1e-8, double rtol = 1e-5) {
  return std::abs(a - b) <= (atol + rtol * std::abs(b));
}

static Eigen::Matrix3d rotZ(double theta) {
  Eigen::Matrix3d m;
  m << std::cos(theta), -std::sin(theta), 0., std::sin(theta), std::cos(theta), 0., 0., 0., 1.;
  return m;
}

namespace elkapod_gait_gen {
using namespace std::chrono_literals;

ElkapodGaitGen::ElkapodGaitGen() : Node("elkapod_gait") {
  trajectory_freq_hz = this->declare_parameter<double>("trajectory.frequency_hz");
  min_swing_time_sec_ = this->declare_parameter<double>("gait.min_swing_time_sec");
  phase_lag_ = this->declare_parameter<double>("gait.default_phase_lag");

  leg_spacing_ = this->declare_parameter<double>("leg_spacing.default");
  step_height_ = this->declare_parameter<double>("gait.step.height.default");

  base_height_ = this->declare_parameter<double>("base_height.default");
  base_height_min_ = this->declare_parameter<double>("base_height.min");
  base_height_max_ = this->declare_parameter<double>("base_height.max");
  set_base_height_ = base_height_;

  const double k_roll = this->declare_parameter<double>("roll.pid.k");
  const double ti_roll = this->declare_parameter<double>("roll.pid.ti");
  const double td_roll = this->declare_parameter<double>("roll.pid.td");
  const double cmd_lo_roll = this->declare_parameter<double>("roll.pid.cmd_lo");
  const double cmd_hi_roll = this->declare_parameter<double>("roll.pid.cmd_hi");
  roll_limit_ = this->declare_parameter<double>("roll.max_rad");

  const double k_pitch = this->declare_parameter<double>("pitch.pid.k");
  const double ti_pitch = this->declare_parameter<double>("pitch.pid.ti");
  const double td_pitch = this->declare_parameter<double>("pitch.pid.td");
  const double cmd_lo_pitch = this->declare_parameter<double>("pitch.pid.cmd_lo");
  const double cmd_hi_pitch = this->declare_parameter<double>("pitch.pid.cmd_hi");
  pitch_limit_ = this->declare_parameter<double>("pitch.max_rad");

  // Subscriptions
  velocity_sub_ = this->create_subscription<VelCmd>(
      "/cmd_vel", 10, std::bind(&ElkapodGaitGen::velocityCallback, this, std::placeholders::_1));

  param_sub_ = this->create_subscription<FloatMsg>(
      "/cmd_base_height", 10,
      std::bind(&ElkapodGaitGen::paramCallback, this, std::placeholders::_1));

  gait_type_sub_ = this->create_subscription<IntMsg>(
      "/cmd_gait_type", 10,
      std::bind(&ElkapodGaitGen::gaitTypeCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<IMUMsg>(
      "/imu", 10, std::bind(&ElkapodGaitGen::imuCallback, this, std::placeholders::_1));

  roll_sub_ = this->create_subscription<FloatMsg>(
      "/roll_setpoint", 10, std::bind(&ElkapodGaitGen::rollCallback, this, std::placeholders::_1));

  pitch_sub_ = this->create_subscription<FloatMsg>(
      "/pitch_setpoint", 10,
      std::bind(&ElkapodGaitGen::pitchCallback, this, std::placeholders::_1));

  fsr_sub_ = this->create_subscription<Int8ArrayMsg>(
      "/legs/fsr_contact", 10, std::bind(&ElkapodGaitGen::fsrCallback, this, std::placeholders::_1));

  // Publishers
  leg_signal_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/elkapod_ik_controller/elkapod_leg_positions", 10);

  // Services
  enable_srv_ = this->create_service<ServiceTriggerSrv>(
      "/gait_gen_enable", std::bind(&ElkapodGaitGen::enableServiceCallback, this,
                                    std::placeholders::_1, std::placeholders::_2));
  disable_srv_ = this->create_service<ServiceTriggerSrv>(
      "/gait_gen_disable", std::bind(&ElkapodGaitGen::disableServiceCallback, this,
                                     std::placeholders::_1, std::placeholders::_2));

  const double write_loop_dt = 1.0 / trajectory_freq_hz;

  leg_clock_timer_ = this->create_timer(std::chrono::duration<double>(write_loop_dt),
                                        std::bind(&ElkapodGaitGen::updateAndWriteCommands, this));
  leg_clock_timer_->cancel();
  leg_path_gen_ = std::make_unique<elkapod_leg_paths::BasicPathBezier>(0.0, step_height_);

  base_link_rotations_ = {0.63973287, -0.63973287, M_PI / 2., -M_PI / 2., 2.38414364, -2.38414364};
  base_link_translations_ = {{0.17841, 0.13276, -0.025},  {0.17841, -0.13276, -0.025},
                             {0.0138, 0.1643, -0.025},    {0.0138, -0.1643, -0.025},
                             {-0.15903, 0.15038, -0.025}, {-0.15903, -0.15038, -0.025}};

  leg_clock_ = std::vector<double>(kLegsNb, 0.);
  leg_phase_ = std::vector<int>(kLegsNb, 0);
  leg_phase_shift_ = std::vector<double>(kLegsNb, 0.);
  phase_offset_ = std::vector<double>(kLegsNb, 0.);

  current_vel_scalar_ = 0.;
  current_angular_velocity_ = 0.;
  cycle_time_ = 0.0;
  ema_filter_alfa_ = 1. - std::exp(-write_loop_dt / EMA_FILTER_TAU);
  base_height_ema_filter_alfa_ = 1. - std::exp(-write_loop_dt / EMA_FILTER_TAU_BASE_HEIGHT);

  current_velocity_ = std::vector<Eigen::Vector2d>(kLegsNb, {0.0, 0.0});
  last_leg_position_ = std::vector<Eigen::Vector3d>(kLegsNb, {0.0, 0.0, 0.0});
  last_leg_position_relative_ = std::vector<Eigen::Vector3d>(kLegsNb, {0.0, 0.0, 0.0});

  roll_pid_ = PID(k_roll, ti_roll, td_roll, write_loop_dt);
  roll_pid_.setCommandLimits(cmd_lo_roll, cmd_hi_roll);

  pitch_pid_ = PID(k_pitch, ti_pitch, td_pitch, write_loop_dt);
  pitch_pid_.setCommandLimits(cmd_lo_pitch, cmd_hi_pitch);

  fsr_data_ = std::vector<double>(6, 0.);

  RCLCPP_INFO(this->get_logger(), "Elkapod gait generator initialized. Use service to activate.");
}

void ElkapodGaitGen::init() {
  leg_path_gen_->init();
  changeGait();
  leg_clock_timer_->reset();
  init_time_ = this->now();
  state_ = State::IDLE;
  RCLCPP_INFO(this->get_logger(), "Elkapod gait generator started.");
}

void ElkapodGaitGen::deinit() {
  leg_clock_timer_->cancel();
  state_ = State::DISABLED;
}

void ElkapodGaitGen::enableServiceCallback([[maybe_unused]] ServiceTriggerSrv_Req request,
                                           ServiceTriggerSrv_Resp response) {
  if (state_ == State::DISABLED) {
    init();
    response->success = true;
    response->message = "Gait enabled.";
  } else {
    response->success = false;
    response->message = "Already enabled.";
  }
}

void ElkapodGaitGen::disableServiceCallback([[maybe_unused]] ServiceTriggerSrv_Req request,
                                            ServiceTriggerSrv_Resp response) {
  if (state_ == State::DISABLED) {
    response->success = false;
    response->message = "Already disabled.";
  } else if (state_ == State::WALKING) {
    response->success = false;
    response->message = "Cannot disable while walking.";
  } else {
    deinit();
    response->success = true;
    response->message = "Gait disabled.";
  }
}

void ElkapodGaitGen::gaitTypeCallback(const IntMsg::SharedPtr msg) {
  if (state_ == State::IDLE) {
    if (msg->data == 0) {
      gait_type_ = GaitType::WAVE;
      changeGait();
      RCLCPP_INFO(this->get_logger(), "Gait set to WAVE");
    } else if (msg->data == 1) {
      gait_type_ = GaitType::RIPPLE;
      changeGait();
      RCLCPP_INFO(this->get_logger(), "Gait set to RIPPLE");
    } else if (msg->data == 2) {
      gait_type_ = GaitType::TRIPOD;
      changeGait();
      RCLCPP_INFO(this->get_logger(), "Gait set to TRIPOD");
    }
  }
}

void ElkapodGaitGen::paramCallback(const FloatMsg::SharedPtr msg) {
  if (msg->data >= base_height_min_ && msg->data <= base_height_max_) {
    set_base_height_ = msg->data;
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Couldn't set new base height goal - value out of allowed range");
  }
}

void ElkapodGaitGen::imuCallback(const IMUMsg::SharedPtr msg) {
  tf2::fromMsg(msg->orientation, q_);
  q_.normalize();
  tf2::Matrix3x3(q_).getRPY(roll_, pitch_, yaw_);
}

void ElkapodGaitGen::rollCallback(const FloatMsg::SharedPtr msg) {
  auto x = std::pow(msg->data, 2) / std::pow(roll_limit_, 2) +
           std::pow(set_pitch_, 2) / std::pow(pitch_limit_, 2);

  if (x <= 1.0) {
    set_roll_ = msg->data;
  } else {
    RCLCPP_WARN(get_logger(), "Couldn't set new roll value - out of allowed range");
  }
}

void ElkapodGaitGen::pitchCallback(const FloatMsg::SharedPtr msg) {
  auto x = std::pow(set_roll_, 2) / std::pow(roll_limit_, 2) +
           std::pow(msg->data, 2) / std::pow(pitch_limit_, 2);

  if (x <= 1.0) {
    set_pitch_ = msg->data;
  } else {
    RCLCPP_WARN(get_logger(), "Couldn't set new pitch value - out of allowed range");
  }
}

void ElkapodGaitGen::fsrCallback(const Int8ArrayMsg::SharedPtr msg) {
  std::copy(msg->data.cbegin(), msg->data.cend(), fsr_data_.begin());
}

void ElkapodGaitGen::velocityCallback(const VelCmd::SharedPtr msg) {
  if ((!is_close(msg->linear.x, 0.0, INPUT_VEL_TOL) ||
       !is_close(msg->linear.y, 0.0, INPUT_VEL_TOL)) &&
      !is_close(msg->angular.z, 0.0, INPUT_VEL_TOL)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                         "Invalid Twist message! Cannot apply both linear and angular velocities!");
  } else {
    received_vel_command_ = *msg;
  }
}

void ElkapodGaitGen::velocityDeadzone(Eigen::Vector2d& vel, double& angular_vel) {
  const double linear_vel = vel.norm();
  if (linear_vel < deadzone_d_) {
    vel.setZero();
  }

  if (std::fabs(angular_vel) < deadzone_d_) {
    angular_vel = 0.0;
  }
}

void ElkapodGaitGen::velocityClamp(Eigen::Vector2d& vel, double& angular_vel) {
  const double linear_vel = vel.norm();
  if (linear_vel > max_vel_ && linear_vel > 0) {
    vel *= max_vel_ / linear_vel;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        std::format(
            "Provided velocity exceeds max velocity of {:.3f} m / s! Clamping to the limit...",
            max_vel_)
            .c_str());
  }

  if (std::fabs(angular_vel) > max_angular_vel_ && std::fabs(angular_vel) > 0) {
    angular_vel *= max_angular_vel_ / std::fabs(angular_vel);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                         std::format("Provided angular velocity exceeds max velocity of {:.3f} rad "
                                     "/ s! Clamping to the limit...",
                                     max_angular_vel_)
                             .c_str());
  }
}
void ElkapodGaitGen::updateVelocityCommand() {
  Eigen::Vector2d vel_command =
      Eigen::Vector2d({received_vel_command_.linear.x, received_vel_command_.linear.y});
  double angular_vel = received_vel_command_.angular.z;

  velocityDeadzone(vel_command, angular_vel);
  velocityClamp(vel_command, angular_vel);

  current_vel_command_ =
      current_vel_command_ + ema_filter_alfa_ * (vel_command - current_vel_command_);
  current_angular_velocity_ =
      current_angular_velocity_ + ema_filter_alfa_ * (angular_vel - current_angular_velocity_);

  if (!is_close(angular_vel, 0.0, VEL_TOL)) {
    for (auto& leg_vel : current_velocity_) {
      leg_vel.setZero();
    }

    constexpr double kBaseToLegMountDist = 0.20204;
    const double R = leg_spacing_ + kBaseToLegMountDist;
    step_length_ = cycle_time_ * duty_factor_ * R * std::fabs(angular_vel);
  } else {
    for (auto& leg_vel : current_velocity_) {
      leg_vel = current_vel_command_;
    }

    current_vel_scalar_ = current_vel_command_.norm();
    const double T_stride = cycle_time_ * (duty_factor_ - phase_lag_);
    step_length_ = current_vel_scalar_ * T_stride;

    if (gait_type_ == GaitType::TRIPOD) {
      step_length_ /= 0.7;
    }
  }

  RCLCPP_DEBUG(get_logger(), std::format("Current velocity: {:.4f} m/s\tAngular: {:.4f} rad/s",
                                         current_vel_scalar_, current_angular_velocity_)
                                 .c_str());

  if (is_close(current_vel_scalar_, 0.0, VEL_TOL) &&
      is_close(current_angular_velocity_, 0.0, VEL_TOL) && state_ == State::WALKING) {
    RCLCPP_INFO(this->get_logger(), "Going to IDLE state");
    state_ = State::IDLE;
  } else if ((!is_close(current_vel_scalar_, 0.0, VEL_TOL) ||
              !is_close(current_angular_velocity_, 0.0, VEL_TOL)) &&
             state_ == State::IDLE) {
    RCLCPP_INFO(this->get_logger(), "Going to WALKING state");
    init_time_ = this->now();
    state_ = State::WALKING;
    std::fill(leg_phase_shift_.begin(), leg_phase_shift_.end(), 0.);
  }
}

void ElkapodGaitGen::changeGait() {
  max_vel_ = max_vel_dict_[gait_type_];
  max_angular_vel_ = max_angular_vel_dict_[gait_type_];
  cycle_time_ = cycle_time_dict_[gait_type_];

  if (gait_type_ == GaitType::TRIPOD) {
    duty_factor_ = 1 / 2.;
    phase_offset_ = {0.0, cycle_time_ / 2., cycle_time_ / 2., 0.0, 0.0, cycle_time_ / 2.};

  } else if (gait_type_ == GaitType::RIPPLE || gait_type_ == GaitType::WAVE) {
    phase_offset_.assign(6, 0.0);
    std::array<int, 6> order = {0, 3, 4, 1, 2, 5};
    for (size_t k = 0; k < 6; ++k) phase_offset_[order[k]] = k * cycle_time_ / 6.0;
    if (gait_type_ == GaitType::WAVE) duty_factor_ = 5 / 6.;
    if (gait_type_ == GaitType::RIPPLE) duty_factor_ = 4 / 6.;
  }
}

void ElkapodGaitGen::clockFunction(double t, double T, double phase_shift, int leg_nb) {
  if (!is_close(leg_phase_shift_[leg_nb], phase_shift, 1e-3)) {
    leg_phase_shift_[leg_nb] = phase_shift * std::fmod(t, T);
  }

  const double t_mod = std::fmod((t + 3 * T / 4. + leg_phase_shift_[leg_nb]), T);

  if (t_mod < T * phase_lag_ * 0.5) {  // swing lag phase
    leg_phase_[leg_nb] = 1;
    leg_clock_[leg_nb] = 0.;
  } else if (T * phase_lag_ * 0.5 <= t_mod &&
             t_mod < T * (1. - duty_factor_) - T * phase_lag_ * 0.5) {  // swing
    leg_phase_[leg_nb] = 1;
    leg_clock_[leg_nb] =
        (t_mod - T * phase_lag_ * 0.5) / (T * (1. - duty_factor_) - T * phase_lag_ * 0.5);
  } else if (T * (1. - duty_factor_) - T * phase_lag_ * 0.5 <= t_mod &&
             t_mod < T * (1. - duty_factor_) + 0.5 * T * phase_lag_) {  // swing - stance lag phase
    leg_phase_[leg_nb] = 0;
    leg_clock_[leg_nb] = 0.;
  } else if (T * (1. - duty_factor_) + 0.5 * T * phase_lag_ < t_mod &&
             t_mod < T - 0.5 * T * phase_lag_) {  // stance
    leg_phase_[leg_nb] = 0;
    leg_clock_[leg_nb] = (t_mod - T * (1. - duty_factor_)) / (T - T * (1. - duty_factor_));
  } else {  // stance lag phase
    leg_phase_[leg_nb] = 0;
    leg_clock_[leg_nb] = 1.;
  }
}

void ElkapodGaitGen::updateAndWriteCommands() {
  updateVelocityCommand();

  auto status = leg_path_gen_->updateBasicParameters(step_length_, step_height_);
  if (status.has_value()) {
    leg_path_gen_->updateBasicParameters(0.0, 0.0);
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Error setting new step_length and step_height! Fallback to zero values...");
  }

  if (state_ == State::IDLE) {
    for (size_t leg_nb = 0; leg_nb < kLegsNb; ++leg_nb) {
      leg_phase_[leg_nb] = 0.0;
      leg_clock_[leg_nb] = 0.0;
    }
  } else {
    rclcpp::Time now = get_clock()->now();
    double elapsed_time_sec = (now - init_time_).nanoseconds() / 1e9;
    for (int leg_nb = 0; leg_nb < 6; ++leg_nb) {
      clockFunction(elapsed_time_sec, cycle_time_, phase_offset_[leg_nb], leg_nb);
    }
  }

  auto msg = FloatArrayMsg();
  msg.data.resize(kJointsNum, 0.0);

  // Set height
  base_height_ = base_height_ + base_height_ema_filter_alfa_ * (set_base_height_ - base_height_);

  if (state_ == State::WALKING || state_ == State::IDLE) {
    for (size_t leg_nb = 0; leg_nb < kLegsNb; ++leg_nb) {
      Eigen::Vector3d p;

      if (state_ == State::WALKING) {
        p = leg_path_gen_->eval(leg_clock_[leg_nb], leg_phase_[leg_nb])
                .value_or(Eigen::Vector3d::Zero());

        const double omega = current_angular_velocity_;
        Eigen::Vector2d last_leg_xy(last_leg_position_[leg_nb][0], last_leg_position_[leg_nb][1]);
        Eigen::Vector2d angular_part = omega * Eigen::Vector2d(-last_leg_xy[1], last_leg_xy[0]);

        Eigen::Vector2d vel2D = current_velocity_[leg_nb] + angular_part;

        double angle = std::atan2(vel2D[1], vel2D[0]);
        p = rotZ(angle) * p;
      } else {
        p = Eigen::Vector3d::Zero();
        if(abs((-base_height_ + base_link_translations_[leg_nb][2]) - last_leg_position_relative_[leg_nb][2]) > 0.05){
          p[2] = last_leg_position_relative_[leg_nb][2] + base_height_ - base_link_translations_[leg_nb][2] - 0.05;
        }
      }

      p = rotZ(-base_link_rotations_[leg_nb]) * p;
      p += Eigen::Vector3d(leg_spacing_, 0.0, -base_height_ + base_link_translations_[leg_nb][2]);

      last_leg_position_relative_[leg_nb] = p;
    }
  }

  // Pitch && Roll PIDs
  double e_roll = set_roll_ - roll_;
  double u_roll = roll_pid_.update(e_roll);

  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      std::format("PID roll u: {:.3f}\te: {:.3f}\ty_zad: {:.3f}", u_roll, e_roll, set_roll_)
          .c_str());

  double e_pitch = set_pitch_ - pitch_;
  double u_pitch = pitch_pid_.update(e_pitch);

  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      std::format("PID pitch u: {:.3f}\te: {:.3f}\ty_zad: {:.3f}", u_pitch, e_pitch, set_pitch_)
          .c_str());

  for (size_t i = 0; i < 6; ++i) {
    auto p = last_leg_position_relative_[i];
    const double rot_z = base_link_rotations_[i];
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H.block<3, 3>(0, 0) = Eigen::AngleAxisd(rot_z, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    H.block<3, 1>(0, 3) = base_link_translations_[i];

    Eigen::Vector4d p_homogeneous(p[0], p[1], p[2], 1.0);
    Eigen::Vector4d p_base_homogeneous = H * p_homogeneous;

    double dz = 0.0;
    dz += -u_roll * p_base_homogeneous[1];
    dz += u_pitch * p_base_homogeneous[0];
    p[2] += dz;
    p[2] = std::clamp(-0.2, -0.11, p[2]);
    p_base_homogeneous[2] = p[2];

    last_leg_position_[i] = p_base_homogeneous.head<3>();
    last_leg_position_relative_[i] = p;

    msg.data[i * 3 + 0] = p[0];
    msg.data[i * 3 + 1] = p[1];
    msg.data[i * 3 + 2] = p[2];
  }
  leg_signal_pub_->publish(msg);
}

}  // namespace elkapod_gait_gen
