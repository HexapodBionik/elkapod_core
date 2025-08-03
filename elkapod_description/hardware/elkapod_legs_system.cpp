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

//#define ELKAPOD_4LEGS_TESTS

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

  // UART
  const std::string uart_device = info_.hardware_parameters["uart_device"];
  const uint32_t uart_baudrate = static_cast<uint32_t>(std::stoul(info_.hardware_parameters["uart_baudrate"]));
  const int uart_timeout_ms = std::stoi(info_.hardware_parameters["uart_timeout_ms"]);

  // SPI
  const int spi_bus = std::stoi(info_.hardware_parameters["spi_bus"]);
  const int spi_cs = std::stoi(info_.hardware_parameters["spi_cs"]);
  const int spi_mode = std::stoi(info_.hardware_parameters["spi_mode"]);
  const uint32_t spi_speed_hz = static_cast<uint32_t>(std::stoul(info_.hardware_parameters["spi_speed_hz"]));
  constexpr int spi_bits = 8;

  std::unique_ptr<elkapod_comm::UARTDevice> uart;
  try{
    uart = std::make_unique<elkapod_comm::UARTDevice>(uart_device, uart_baudrate, uart_timeout_ms);
  } catch(const std::runtime_error& error){
    RCLCPP_FATAL(
        get_logger(), error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::unique_ptr<elkapod_comm::SpiDevice> spi;
  try{
    spi = std::make_unique<elkapod_comm::SpiDevice>(spi_bus, spi_cs);

    spi->setMode(spi_mode);
    spi->setSpeed(spi_speed_hz);
    spi->setBitsPerWord(spi_bits);

  } catch(const std::runtime_error& error){
    RCLCPP_FATAL(
        get_logger(), error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  this->comm_ = std::make_unique<elkapod_comm::ElkapodComm>(std::move(uart), std::move(spi));

  // TODO check joint interfaces and sensor interfaces config

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ElkapodLegsSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < positions_.size(); ++i) {
    std::string joint_name = std::format("leg{}_J{}", i / 3 + 1, i % 3 + 1);
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &positions_[i]));
  }

  // Temperature
  state_interfaces.emplace_back(hardware_interface::StateInterface("control_module_temp", hardware_interface::HW_IF_TEMPERATURE, &temperature_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ElkapodLegsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < cmd_positions_.size(); ++i) {
    std::string joint_name = std::format("leg{}_J{}", i / 3 + 1, i % 3 + 1);
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_POSITION, &cmd_positions_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ElkapodLegsSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  comm_->connect();
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
  comm_->sendSystemStartCommand();
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ElkapodLegsSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  comm_->sendSystemShutdownCommand();
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
    [i = 0](double val) mutable {
      float degrees = static_cast<float>(val * 180.0 / M_PI);  // rad -> deg

      if (i % 3 == 0 || i % 3 == 1){
        degrees = std::max(std::min(degrees, 90.0f), -90.0f);
        degrees += 90.0f;
      }
      else if (i % 3 == 2){
        degrees = std::max(std::min(degrees, 0.0f), -180.0f);
        degrees *= -1.0f;
      }

      if(i % 6 == 4 || i % 6 == 5){
        degrees = 180.0f - degrees;
      }

    return i++, degrees;
  });

  #ifdef ELKAPOD_4LEGS_TESTS
    cmd_positions_float[6] = cmd_positions_float[12];
    cmd_positions_float[7] = cmd_positions_float[13];
    cmd_positions_float[8] = cmd_positions_float[14];
    cmd_positions_float[9] = cmd_positions_float[15];
    cmd_positions_float[10] = cmd_positions_float[16];
    cmd_positions_float[11] = cmd_positions_float[17];
  #endif
  
  elkapod_comm::SpiTransmissionRequest request = {
    .angles = cmd_positions_float
  };

  elkapod_comm::SpiTransmissionResponse response = comm_->transfer(request);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Temperature: %2.2f*C", response.temp);

  temperature_ = response.temp;
  return hardware_interface::return_type::OK;
}

}  // namespace elkapod_legs_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  elkapod_legs_system::ElkapodLegsSystemHardware, hardware_interface::SystemInterface)