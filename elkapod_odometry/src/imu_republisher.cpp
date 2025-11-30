#include "../include/elkapod_odometry/imu_republisher.hpp"

namespace imu_republisher {
using namespace std::chrono_literals;

constexpr double pitch_noise_stddev = 0.0005;
constexpr double roll_noise_stddev = 0.0005;
constexpr double yaw_noise_stddev = 0.0005;

constexpr double pitch_noise_bias = 0.005;
constexpr double roll_noise_bias = 0.005;
constexpr double yaw_noise_bias = 0.005;

ImuRepublisher::ImuRepublisher() : Node("imu_republisher") {
  noise_dist_.push_back(std::normal_distribution<double>(pitch_noise_bias, pitch_noise_stddev));
  noise_dist_.push_back(std::normal_distribution<double>(roll_noise_bias, roll_noise_stddev));
  noise_dist_.push_back(std::normal_distribution<double>(yaw_noise_bias, yaw_noise_stddev));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_raw", 10, std::bind(&ImuRepublisher::imuCallback, this, std::placeholders::_1));

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

  RCLCPP_INFO(get_logger(), "IMU republisher started");
}

void ImuRepublisher::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  tf2::Quaternion q_orig;
  tf2::fromMsg(msg->orientation, q_orig);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

  roll += noise_dist_[0](generator_);
  pitch += noise_dist_[1](generator_);
  yaw += noise_dist_[2](generator_);

  tf2::Quaternion q_new;
  q_new.setRPY(roll, pitch, yaw);
  q_new.normalize();

  msg->orientation = tf2::toMsg(q_new);
  msg->orientation_covariance = {std::pow(pitch_noise_stddev, 2), 0.0, 0.0, 0.0,
                                 std::pow(roll_noise_stddev, 2),  0.0, 0.0, 0.0,
                                 std::pow(yaw_noise_stddev, 2)};

  msg->header.frame_id = "imu_link";
  imu_pub_->publish(*msg);
}
}  // namespace imu_republisher