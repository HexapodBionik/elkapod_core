#include "elkapod_gait_controller/elkapod_gait_controller.hpp"

#include <eigen3/Eigen/Eigen>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace {
constexpr auto DEFAULT_COMMAND_TOPIC = "~/elkapod_leg_positions";
static inline float deg2rad(float deg) { return deg / 180.f * M_PI; }
}  // namespace

namespace elkapod_gait_controller {
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;

ElkapodGaitController::ElkapodGaitController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ElkapodGaitController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Eigen::Vector3d m1(params_.m1[0], params_.m1[1], params_.m1[2]);
  // Eigen::Vector3d a1(params_.a1[0], params_.a1[1], params_.a1[2]);
  // Eigen::Vector3d a2(params_.a2[0], params_.a2[1], params_.a2[2]);
  // Eigen::Vector3d a3(params_.a3[0], params_.a3[1], params_.a3[2]);



  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration ElkapodGaitController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.joints) {
    conf_names.push_back(joint_name);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration ElkapodGaitController::state_interface_configuration() const {
  return {interface_configuration_type::NONE, {}};
}

controller_interface::return_type ElkapodGaitController::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
  auto logger = get_node()->get_logger();
  // if (command_msg_.data.size() < 18) {
  //   return controller_interface::return_type::OK;
  // }

  std::array<double, 18> output = {0.1, 0.0, -0.12, 0.1, 0.0, -0.12, 0.1, 0.0, -0.12, 0.1, 0.0, -0.12, 0.1, 0.0, -0.12, 0.1, 0.0, -0.12};

  // Eigen::Vector3d input;
  // for (int i = 0; i < 6; ++i) {
  //   input[0] = command_msg_.data[i * 3];
  //   input[1] = command_msg_.data[i * 3 + 1];
  //   input[2] = command_msg_.data[i * 3 + 2];

  //   Eigen::Vector3d anglesDeg = solver_->inverse(input);

  //   if (anglesDeg.array().isNaN().any()) {
  //     RCLCPP_ERROR(logger, "Inverse kinematics error while processing input for leg %d", i + 1);
  //     RCLCPP_ERROR(
  //         logger, std::format("Input: {:.3f} {:.3f} {:.3f}", input[0], input[1], input[2]).c_str());
  //   } else {
  //     output[i * 3] = anglesDeg[0];
  //     output[i * 3 + 1] = anglesDeg[1];
  //     output[i * 3 + 2] = anglesDeg[2];

  //     std::string msg2 =
  //         std::format("Angles for leg {} theta0: {:.3f} theta1: {:.3f} theta2: {:.3f}", i + 1,
  //                     output[i * 3], output[i * 3 + 1], output[i * 3 + 2]);
  //     RCLCPP_DEBUG(logger, msg2.c_str());
  //   }
  // }

  for (size_t i = 0; i < 18; ++i) {
    (void)command_interfaces_[i].set_value(output[i]);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ElkapodGaitController::on_configure(
    const rclcpp_lifecycle::State &) {
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->try_update_params(params_)) {
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  // initialize command subscriber
  // position_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
  //     DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
  //     [this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) -> void {
  //       if (!subscriber_is_active_) {
  //         RCLCPP_WARN(get_node()->get_logger(),
  //                     "Can't accept new commands. subscriber is inactive");
  //         return;
  //       }

  //       received_position_msg_.set(*msg);
  //     });

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodGaitController::on_activate(
    const rclcpp_lifecycle::State &) {
  // subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodGaitController::on_deactivate(
    const rclcpp_lifecycle::State &) {
  // subscriber_is_active_ = false;
  halt();
  reset_buffers();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodGaitController::on_cleanup(
    const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodGaitController::on_error(
    const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

bool ElkapodGaitController::reset() { return true; }

void ElkapodGaitController::reset_buffers() {}

void ElkapodGaitController::halt() {}

}  // namespace elkapod_ik_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(elkapod_gait_controller::ElkapodGaitController,
                       controller_interface::ControllerInterface)