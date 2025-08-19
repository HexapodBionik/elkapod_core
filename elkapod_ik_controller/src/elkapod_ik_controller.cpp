#include "elkapod_ik_controller/elkapod_ik_controller.hpp"

#include <eigen3/Eigen/Eigen>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace {
constexpr auto DEFAULT_COMMAND_TOPIC = "~/elkapod_leg_positions";
static inline float deg2rad(float deg) { return deg / 180.f * M_PI; }
}  // namespace

namespace elkapod_ik_controller {
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;

ElkapodIKController::ElkapodIKController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn ElkapodIKController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  Eigen::Vector3d m1(0.0, 0.0, 0.025);
  Eigen::Vector3d a1(0.0676, 0.0, 0.0);
  Eigen::Vector3d a2(0.09237, 0.0, 0.0);
  Eigen::Vector3d a3(0.22524, 0.0, 0.0);

  const std::vector<Eigen::Vector3d> input = {m1, a1, a2, a3};
  solver_ = std::make_shared<KinematicsSolver>(input);

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration ElkapodIKController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.joints) {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration ElkapodIKController::state_interface_configuration() const {
  return {interface_configuration_type::NONE, {}};
}

controller_interface::return_type ElkapodIKController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  auto logger = get_node()->get_logger();

  auto current_ref_op = received_position_msg_.try_get();

  if (current_ref_op.has_value()) {
    command_msg_ = current_ref_op.value();
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type ElkapodIKController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  auto logger = get_node()->get_logger();
  if (command_msg_.data.size() < 18) {
    return controller_interface::return_type::OK;
  }

  std::array<double, 18> output;

  Eigen::Vector3d input;
  for (int i = 0; i < 6; ++i) {
    input[0] = command_msg_.data[i * 3];
    input[1] = command_msg_.data[i * 3 + 1];
    input[2] = command_msg_.data[i * 3 + 2];

    Eigen::Vector3d anglesDeg = solver_->inverse(input);
    output[i * 3] = deg2rad(anglesDeg[0]);
    output[i * 3 + 1] = deg2rad(anglesDeg[1]);
    output[i * 3 + 2] = deg2rad(anglesDeg[2]);
  }

  // if (reference_interfaces_.size() != 18) {
  //   reference_interfaces_.assign(18, 0.0);
  // }
  for (size_t i = 0; i < 18; ++i) {
    (void)command_interfaces_[i].set_value(output[i]);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ElkapodIKController::on_configure(
    const rclcpp_lifecycle::State &) {
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->try_update_params(params_)) {
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  // initialize command subscriber
  position_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) -> void {
        if (!subscriber_is_active_) {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Can't accept new commands. subscriber is inactive");
          return;
        }

        received_position_msg_.set(*msg);
      });

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodIKController::on_activate(
    const rclcpp_lifecycle::State &) {
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodIKController::on_deactivate(
    const rclcpp_lifecycle::State &) {
  subscriber_is_active_ = false;
  halt();
  reset_buffers();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodIKController::on_cleanup(
    const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodIKController::on_error(
    const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

bool ElkapodIKController::reset() { return true; }

void ElkapodIKController::reset_buffers() {}

void ElkapodIKController::halt() {}

bool ElkapodIKController::on_set_chained_mode(bool /*chained_mode*/) { return false; }

std::vector<hardware_interface::CommandInterface>
ElkapodIKController::on_export_reference_interfaces() {
  if (reference_interfaces_.size() != params_.joints.size()) {
    reference_interfaces_.assign(params_.joints.size(), 0.0);
  }

  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve(params_.joints.size());

  const std::string ctrl_name = this->get_node()->get_name();

  for (std::size_t i = 0; i < params_.joints.size(); ++i) {
    std::string coordinate;
    if (i % 3 == 0) {
      coordinate = 'x';
    } else if (i % 3 == 1) {
      coordinate = 'y';
    } else {
      coordinate = 'z';
    }
    refs.emplace_back(std::format("{}/leg{}_{}", ctrl_name, i / 3 + 1, coordinate), "point",
                      &reference_interfaces_[i]);
  }

  return refs;
}

}  // namespace elkapod_ik_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(elkapod_ik_controller::ElkapodIKController,
                       controller_interface::ChainableControllerInterface)