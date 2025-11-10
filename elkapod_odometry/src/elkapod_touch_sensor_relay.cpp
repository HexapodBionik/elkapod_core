#include "../include/elkapod_odometry/elkapod_touch_sensor_relay.hpp"

ElkapodTouchSensorRelay::ElkapodTouchSensorRelay() : Node("elkapod_fsr_relay") {
  fsr_sub_1_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/legs/leg1_fsr", 10,
      std::bind(&ElkapodTouchSensorRelay::fsrCallback, this, std::placeholders::_1));

  fsr_sub_2_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/legs/leg2_fsr", 10,
      std::bind(&ElkapodTouchSensorRelay::fsrCallback, this, std::placeholders::_1));

  fsr_sub_3_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/legs/leg3_fsr", 10,
      std::bind(&ElkapodTouchSensorRelay::fsrCallback, this, std::placeholders::_1));

  fsr_sub_4_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/legs/leg4_fsr", 10,
      std::bind(&ElkapodTouchSensorRelay::fsrCallback, this, std::placeholders::_1));

  fsr_sub_5_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/legs/leg5_fsr", 10,
      std::bind(&ElkapodTouchSensorRelay::fsrCallback, this, std::placeholders::_1));

  fsr_sub_6_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/legs/leg6_fsr", 10,
      std::bind(&ElkapodTouchSensorRelay::fsrCallback, this, std::placeholders::_1));

  my_publisher_ = this->create_publisher<elkapod_msgs::msg::Float64ArrayStamped>("/legs/fsr", 10);

  RCLCPP_INFO(get_logger(), "Elkapod FSR relay started!");
}

void ElkapodTouchSensorRelay::fsrCallback(
    const ros_gz_interfaces::msg::Contacts::SharedPtr contacts) {
  std::vector<double> fsr_values(6, 0.);
  for (size_t i = 0; i < 6; ++i) {
    if (std::format("Elkapod::leg{}_FCP::leg{}_FCP_collision", i + 1, i + 1) ==
        contacts->contacts[0].collision1.name) {
      fsr_values[i] = std::fabs(contacts->contacts[0].wrenches[0].body_2_wrench.force.z);
      break;
    }
  }
  auto msg = elkapod_msgs::msg::Float64ArrayStamped();
  msg.header.stamp = get_clock()->now();
  msg.data = fsr_values;
  my_publisher_->publish(msg);
}