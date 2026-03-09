#include "../include/elkapod_odometry/elkapod_binary_fsr_publisher.hpp"

ElkapodBinaryFsrPublisher::ElkapodBinaryFsrPublisher() : Node("elkapod_binary_fsr_publisher") {
  fsr_sub_ = this->create_subscription<elkapod_msgs::msg::Float64ArrayStamped>(
      "/legs/fsr", 10,
      std::bind(&ElkapodBinaryFsrPublisher::fsrCallback, this, std::placeholders::_1));

  threshold_ = declare_parameter<double>("threshold");
  reset_delay_ = declare_parameter<double>("reset_delay");

  my_publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("/legs/fsr_contact", 10);

  last_contact_ = {0.};
  RCLCPP_INFO(get_logger(), "Elkapod binary fsr publisher started!");
  RCLCPP_INFO(
      get_logger(),
      std::format("Threshold: {:.5f}\tReset delay: {:.5f}s", threshold_, reset_delay_).c_str());
}

void ElkapodBinaryFsrPublisher::fsrCallback(
    const elkapod_msgs::msg::Float64ArrayStamped fsr_values) {
  auto msg = std_msgs::msg::Int8MultiArray();
  std::vector<int8_t> data(6, 0.);
  for (size_t i = 0; i < 6; ++i) {
    if (fsr_values.data[i] >= threshold_) {
      data[i] = 1;
      last_contact_[i] = get_clock()->now().seconds();
    } else if (fsr_values.data[i] < threshold_ &&
               get_clock()->now().seconds() - last_contact_[i] < reset_delay_) {
      data[i] = 1;
    }
  }
  msg.data = data;
  my_publisher_->publish(msg);
}