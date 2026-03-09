#include "../include/elkapod_odometry/elkapod_touch_sensor_relay.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

ElkapodTouchSensorRelay::ElkapodTouchSensorRelay() : Node("elkapod_fsr_relay") {
  fsr_buffer_.assign(6, 0.0);

  auto subscribe_leg = [this](int id, const std::string& topic) {
    return this->create_subscription<ros_gz_interfaces::msg::Contacts>(
        topic, 1, [this, id](const ros_gz_interfaces::msg::Contacts::SharedPtr msg) {
          this->fsrCallback(msg, id);
        });
  };

  fsr_subs_[0] = subscribe_leg(0, "/legs/leg1_fsr");
  fsr_subs_[1] = subscribe_leg(1, "/legs/leg2_fsr");
  fsr_subs_[2] = subscribe_leg(2, "/legs/leg3_fsr");
  fsr_subs_[3] = subscribe_leg(3, "/legs/leg4_fsr");
  fsr_subs_[4] = subscribe_leg(4, "/legs/leg5_fsr");
  fsr_subs_[5] = subscribe_leg(5, "/legs/leg6_fsr");

  my_publisher_ = this->create_publisher<elkapod_msgs::msg::Float64ArrayStamped>("/legs/fsr", 10);
  publish_timer_ =
      this->create_timer(2.5ms, std::bind(&ElkapodTouchSensorRelay::timerCallback, this));

  RCLCPP_INFO(get_logger(), "Elkapod FSR relay started with 100Hz timer!");
}

void ElkapodTouchSensorRelay::fsrCallback(
    const ros_gz_interfaces::msg::Contacts::SharedPtr contacts, int leg_id) {
  if (contacts->contacts.empty()) return;

  double force_z = std::fabs(contacts->contacts[0].wrenches[0].body_2_wrench.force.z);

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    fsr_buffer_[leg_id] = force_z;
  }
}

void ElkapodTouchSensorRelay::timerCallback() {
  auto msg = elkapod_msgs::msg::Float64ArrayStamped();
  msg.header.stamp = this->get_clock()->now();

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    msg.data = fsr_buffer_;
  }

  my_publisher_->publish(msg);
}