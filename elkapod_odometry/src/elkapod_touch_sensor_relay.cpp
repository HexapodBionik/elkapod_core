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

  my_publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("/legs/fsr", 10);

  timer_ = this->create_timer(std::chrono::duration<double>(0.01),
                              std::bind(&ElkapodTouchSensorRelay::collisionPubCallback, this));

  force_cache_ = {0.};
  last_contact_ = {0.};
  RCLCPP_INFO(get_logger(), "Elkapod FSR relay started!");
}

void ElkapodTouchSensorRelay::fsrCallback(
    const ros_gz_interfaces::msg::Contacts::SharedPtr contacts) {
  for (size_t i = 0; i < 6; ++i) {
    if (std::format("Elkapod::leg{}_FCP::leg{}_FCP_collision", i + 1, i + 1) ==
        contacts->contacts[0].collision1.name) {
      force_cache_[i] = std::fabs(contacts->contacts[0].wrenches[0].body_2_wrench.force.z);
      break;
    }
  }
}

void ElkapodTouchSensorRelay::collisionPubCallback() {
  auto msg = std_msgs::msg::Int8MultiArray();
  std::vector<int8_t> data(6, 0.);
  for (size_t i = 0; i < 6; ++i) {
    if (force_cache_[i] >= treshold_) {
      data[i] = 1;
      last_contact_[i] = get_clock()->now().seconds();
    } else if (force_cache_[i] < treshold_ &&
               get_clock()->now().seconds() - last_contact_[i] < 0.02) {
      data[i] = 1;
    }
  }
  msg.data = data;
  my_publisher_->publish(msg);
}