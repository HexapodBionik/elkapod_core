#include "../include/elkapod_gait_gen_cpp/elkapod_gait_gen.hpp"

#include <format>
#include <functional>

namespace {
constexpr auto INPUT_VEL_TOL = 1e-5;
constexpr auto VEL_TOL = 1e-3;

constexpr auto EMA_FILTER_TAU = 0.15;

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
  trajectory_freq_hz = this->declare_parameter<double>("trajectory.frequency_hz", 50.0);
  min_swing_time_sec_ = this->declare_parameter<double>("gait.min_swing_time_sec", 0.42);
  phase_lag_ = this->declare_parameter<double>("gait.default_phase_lag_sec", 0.03);

  leg_spacing_ = this->declare_parameter<double>("leg_spacing", 0.175);

  step_length_ = this->declare_parameter<double>("gait.step.length.default", 0.15);
  step_height_ = this->declare_parameter<double>("gait.step.height.default", 0.05);

  base_height_ = this->declare_parameter<double>("../common.base_height.default", 0.17);
  base_height_min_ = this->declare_parameter<double>("../common.base_height.min", 0.12);
  base_height_max_ = this->declare_parameter<double>("../common.base_height.max", 0.22);
  set_base_height_ = base_height_;

  // Subscriptions
  velocity_sub_ = this->create_subscription<VelCmd>(
      "/cmd_vel", 10, std::bind(&ElkapodGaitGen::velocityCallback, this, std::placeholders::_1));

  param_sub_ = this->create_subscription<FloatMsg>(
      "/cmd_base_param", 10,
      std::bind(&ElkapodGaitGen::paramCallback, this, std::placeholders::_1));

  gait_type_sub_ = this->create_subscription<IntMsg>(
      "/cmd_gait_type", 10,
      std::bind(&ElkapodGaitGen::gaitTypeCallback, this, std::placeholders::_1));

  // Publishers
  leg_signal_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/elkapod_ik_controller/elkapod_leg_positions", 10);
  leg_phase_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/leg_phase_signal", 10);

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
  base_traj_ = std::make_unique<ElkapodLegPath>(step_length_, 0.04);

  base_link_rotations_ = {0.63973287, -0.63973287, M_PI / 2., -M_PI / 2., 2.38414364, -2.38414364};
  base_link_translations_ = {{0.17841, 0.13276, -0.03},  {0.17841, -0.13276, -0.03},
                             {0.0138, 0.1643, -0.03},    {0.0138, -0.1643, -0.03},
                             {-0.15903, 0.15038, -0.03}, {-0.15903, -0.15038, -0.03}};

  leg_clock_ = std::vector<double>(kLegsNb, 0.);
  leg_phase_ = std::vector<int>(kLegsNb, 0);
  leg_phase_shift_ = std::vector<double>(kLegsNb, 0.);
  phase_offset_ = std::vector<double>(kLegsNb, 0.);

  current_vel_scalar_ = 0.;
  current_angular_velocity_ = 0.;
  cycle_time_ = 0.0;
  ema_filter_alfa_ = 1. - std::exp(-write_loop_dt / EMA_FILTER_TAU);

  current_velocity_ = std::vector<Eigen::Vector2d>(kLegsNb, {0.0, 0.0});
  last_leg_position_ = std::vector<Eigen::Vector3d>(kLegsNb, {0.0, 0.0, 0.0});

  RCLCPP_INFO(this->get_logger(), "Elkapod gait generator initialized. Use service to activate.");
}

void ElkapodGaitGen::init() {
  base_traj_->init();
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
  if (msg->data > base_height_min_ && msg->data < base_height_max_) {
    set_base_height_ = msg->data;
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Couldn't set new base height goal - value out of allowed range");
  }
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
    angular_vel *= max_angular_vel_ / angular_vel;
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

    const double R = leg_spacing_ + 0.22;
    step_length_ = cycle_time_ * duty_factor_ * R * fabs(angular_vel);
  } else {
    for (auto& leg_vel : current_velocity_) {
      leg_vel = current_vel_command_;
    }

    current_vel_scalar_ = current_vel_command_.norm();
    step_length_ = cycle_time_ * current_vel_scalar_ * duty_factor_;
  }

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
    if (leg_phase_shift_[leg_nb] < phase_shift) {
      leg_phase_shift_[leg_nb] += 0.05 * phase_shift;
    } else {
      leg_phase_shift_[leg_nb] -= 0.05 * phase_shift;
    }
  }

  const double t_mod = std::fmod((t + 3 * T / 4. + leg_phase_shift_[leg_nb]), T);

  if (t_mod < T * phase_lag_) {  // swing lag phase
    leg_phase_[leg_nb] = 1;
    leg_clock_[leg_nb] = 0.0;
  } else if (T * phase_lag_ < t_mod && t_mod < T * (1. - duty_factor_)) {  // swing
    leg_phase_[leg_nb] = 1;
    leg_clock_[leg_nb] = (t_mod - T * phase_lag_) / (T * (1. - duty_factor_) - T * phase_lag_);
  } else {  // stance
    leg_phase_[leg_nb] = 0;
    leg_clock_[leg_nb] = (t_mod - T * (1. - duty_factor_)) / (T - T * (1. - duty_factor_));
  }
}

void ElkapodGaitGen::updateAndWriteCommands() {
  updateVelocityCommand();

  base_traj_->step_length_ = step_length_;
  base_traj_->init();

  auto msg_phase = FloatArrayMsg();
  msg_phase.data.resize(kLegsNb, 0.0);

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
      msg_phase.data[leg_nb] = leg_phase_[leg_nb];
    }
  }

  // RCLCPP_INFO(get_logger(), std::format("Current velocity: {:.4f} m/s\tAngular: {:.4f} rad/s",
  // current_vel_scalar_, current_angular_velocity_).c_str());

  leg_phase_pub_->publish(msg_phase);

  auto msg3 = FloatArrayMsg();
  msg3.data.resize(kJointsNum, 0.0);

  // TODO Base height regulation commented for now, to be fixed
  // if (!is_close(base_height_, set_base_height_, 1e-3)) {
  //   if (base_height_ < set_base_height_)
  //     base_height_ += 0.005;
  //   else
  //     base_height_ -= 0.005;
  // }

  std::string output = std::format("Cycle time: {:.3f}s Offsets:", cycle_time_);
  for (size_t i = 0; i < leg_phase_shift_.size(); ++i) {
    output += std::format(" {:.3f}/{:.3f}\t", leg_phase_shift_[i], phase_offset_[i]);
  }
  RCLCPP_INFO(get_logger(), output.c_str());

  for (size_t leg_nb = 0; leg_nb < kLegsNb; ++leg_nb) {
    Eigen::Vector3d p;

    if (state_ == State::WALKING) {
      p = (*base_traj_)(leg_clock_[leg_nb], leg_phase_[leg_nb]);

      const double omega = current_angular_velocity_;
      Eigen::Vector2d last_leg_xy(last_leg_position_[leg_nb][0], last_leg_position_[leg_nb][1]);
      Eigen::Vector2d angular_part = -omega * Eigen::Vector2d(-last_leg_xy[1], last_leg_xy[0]);

      Eigen::Vector2d vel2D = current_velocity_[leg_nb] + angular_part;

      double angle = std::atan2(vel2D[1], vel2D[0]);
      p = rotZ(angle) * p;
    } else {
      p = Eigen::Vector3d::Zero();
    }

    p = rotZ(-base_link_rotations_[leg_nb]) * p;

    p += Eigen::Vector3d(leg_spacing_, 0.0, -base_height_);

    double rot = base_link_rotations_[leg_nb];
    Eigen::Vector3d trans = base_link_translations_[leg_nb];

    Eigen::Matrix4d L_B_H = Eigen::Matrix4d::Identity();
    L_B_H(0, 0) = std::cos(rot);
    L_B_H(0, 1) = -std::sin(rot);
    L_B_H(1, 0) = std::sin(rot);
    L_B_H(1, 1) = std::cos(rot);
    L_B_H(0, 3) = -(std::cos(rot) * leg_spacing_ + trans[0]);
    L_B_H(1, 3) = -(std::sin(rot) * leg_spacing_ + trans[1]);
    L_B_H(2, 3) = base_height_;

    Eigen::Vector4d p_homogeneous(p[0], p[1], p[2], 1.0);
    Eigen::Vector4d p_base_homogeneous = L_B_H * p_homogeneous;

    last_leg_position_[leg_nb] = p_base_homogeneous.head<3>();

    msg3.data[leg_nb * 3 + 0] = p[0];
    msg3.data[leg_nb * 3 + 1] = p[1];
    msg3.data[leg_nb * 3 + 2] = p[2];
  }

  leg_signal_pub_->publish(msg3);
}

}  // namespace elkapod_gait_gen
