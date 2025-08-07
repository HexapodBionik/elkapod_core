#ifndef ELKAPOD_LEGS_SYSTEM_HPP
#define ELKAPOD_LEGS_SYSTEM_HPP

#include <memory>
#include <string>
#include <vector>

#include "elkapod_comm.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"

namespace elkapod_legs_system {
class ElkapodLegsSystemHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ElkapodLegsSystemHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  bool on_init_validate_interfaces(const hardware_interface::HardwareInfo& info);

  inline static constexpr size_t kJointsNum = 18;
  inline static constexpr int kWatchdogRestartAttemps = 10;

  std::array<double, kJointsNum> positions_{};
  std::array<double, kJointsNum> cmd_positions_{};

  std::array<double, 4> temperatures_;
  std::array<double, 10> imu_data_;

  double battery_percentage_;
  double battery_voltage_;
  bool battery_present_;

  int watchdog_counter_ = kWatchdogRestartAttemps;
  rcl_duration_value_t watchdog_time_counter_ = 0;

  std::unique_ptr<elkapod_comm::ElkapodComm> comm_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace elkapod_legs_system

#endif  // ELKAPOD_LEGS_SYSTEM_HPP