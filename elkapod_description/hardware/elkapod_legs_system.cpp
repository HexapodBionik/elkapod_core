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

  std::unique_ptr<elkapod_comm::UARTDevice> uart = std::make_unique<elkapod_comm::UARTDevice>();
  std::unique_ptr<elkapod_comm::SpiDevice> spi = std::make_unique<elkapod_comm::SpiDevice>(0, 0);

  spi->setMode(SPI_MODE_2);
  spi->setSpeed(1000000);
  spi->setBitsPerWord(8);

  // TODO - change this hardcoded setup start
  this->comm_ = std::make_unique<elkapod_comm::ElkapodComm>(std::move(uart), std::move(spi));
  // change this hardcoded setup end

  // TODO check interfaces config

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ElkapodLegsSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < positions_.size(); ++i) {
    std::string joint_name = std::format("leg{}_J{}", i / 3 + 1, i % 3 + 1);
    RCLCPP_INFO(rclcpp::get_logger("ElkapodLegsSystemHardware"), "Exporting state interface: %s", joint_name.c_str());
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &positions_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ElkapodLegsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < cmd_positions_.size(); ++i) {
    std::string joint_name = std::format("leg{}_J{}", i / 3 + 1, i % 3 + 1);
    RCLCPP_INFO(rclcpp::get_logger("ElkapodLegsSystemHardware"), "Exporting command interface: %s", joint_name.c_str());
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_POSITION, &cmd_positions_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ElkapodLegsSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  const LibSerial::BaudRate baudrate = LibSerial::BaudRate::BAUD_1152000;
  constexpr const char* const SERIAL_PORT_1 = "/dev/ttyAMA1";
  this->comm_->connect();

  


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

    std::string msg;
    for(size_t i = 0; i < 12; ++i){
      msg += std::format("{:.2f} ", cmd_positions_float[i]);
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, msg.c_str());
  
  elkapod_comm::SpiTransmissionRequest request = {
    .angles = cmd_positions_float
  };

  elkapod_comm::SpiTransmissionResponse response = comm_->transfer(request);


  //int status = this->comm_->sendAngles(cmd_positions_float.data());
  // if(status != 0x8A){
  //     RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "Timeout / error - status: %02x", status);
  // }

  return hardware_interface::return_type::OK;
}

}  // namespace elkapod_legs_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  elkapod_legs_system::ElkapodLegsSystemHardware, hardware_interface::SystemInterface)