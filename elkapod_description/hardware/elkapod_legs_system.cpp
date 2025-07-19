#include "elkapod_legs_system/elkapod_legs_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <format>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace elkapod_legs_system
{
hardware_interface::CallbackReturn ElkapodLegsSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO - change this hardcoded setup start
  this->comm_ = std::make_unique<ElkapodComm>();
  // change this hardcoded setup end

  // TODO check interfaces config

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ElkapodLegsSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < positions_.size(); ++i) {
    std::string joint_name = std::format("leg{}_J{}", i / 3 + 1, i % 3);
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &positions_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ElkapodLegsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < cmd_positions_.size(); ++i) {
    std::string joint_name = std::format("leg{}_J{}", i / 3 + 1, i % 3);
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_POSITION, &cmd_positions_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ElkapodLegsSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  const LibSerial::BaudRate baudrate = LibSerial::BaudRate::BAUD_115200;
  constexpr const char* const SERIAL_PORT_1 = "/dev/ttyAMA1";
  this->comm_->connect(SERIAL_PORT_1, baudrate);

  

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ElkapodLegsSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  this->comm_->disconnect();

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn ElkapodLegsSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Activating ...please wait...");



  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ElkapodLegsSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");


  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ElkapodLegsSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < cmd_positions_.size(); ++i) {
    positions_[i] = cmd_positions_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ElkapodLegsSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  std::array<float, 18> cmd_positions_float;

  std::transform(
    cmd_positions_.begin(), cmd_positions_.end(),
    cmd_positions_float.begin(),
    [](double val) { return static_cast<float>(val); });

  this->comm_->sendAngles(cmd_positions_float.data());

}

}  // namespace elkapod_legs_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  elkapod_legs_system::ElkapodLegsSystemHardware, hardware_interface::SystemInterface)