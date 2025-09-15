#include "rpi5_gpio_control/rpi5_valve_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace rpi5_gpio_control {

controller_interface::InterfaceConfiguration
RpiValveController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL, interfaces_};
}

controller_interface::InterfaceConfiguration
RpiValveController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn RpiValveController::on_init() {
  auto node = get_node();
  if (!node) return controller_interface::CallbackReturn::ERROR;

  // Parámetros opcionales
  if (node->has_parameter("topic"))
    topic_ = node->get_parameter("topic").as_string();

  if (node->has_parameter("interfaces"))
    interfaces_ = node->get_parameter("interfaces").as_string_array();
  else
    interfaces_ = { "valves/valve0/level","valves/valve1/level","valves/valve2/level","valves/valve3/level","valves/valve4/level" };

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RpiValveController::on_activate(const rclcpp_lifecycle::State &) {
  auto node = get_node();
  if (!node) return controller_interface::CallbackReturn::ERROR;
  sub_ = node->create_subscription<std_msgs::msg::UInt8>(
    topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&RpiValveController::cb, this, std::placeholders::_1));
    last_.store(0);
    bool ok_all = true;
    for (auto & ci : command_interfaces_) {
      ok_all &= ci.set_value(0.0);   // ← comprobar retorno
    }
    if (!ok_all) {
      RCLCPP_WARN(get_node()->get_logger(), "Algunos set_value(0.0) fallaron en on_activate()");
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RpiValveController::on_deactivate(const rclcpp_lifecycle::State &) {
    sub_.reset();
    bool ok_all = true;
    for (auto & ci : command_interfaces_) {
      ok_all &= ci.set_value(0.0);   // ← comprobar retorno
    }
    if (!ok_all) {
      RCLCPP_WARN(get_node()->get_logger(), "Algunos set_value(0.0) fallaron en on_deactivate()");
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

void RpiValveController::cb(const std_msgs::msg::UInt8 & msg) {
  last_.store(msg.data);
}

controller_interface::return_type
RpiValveController::update(const rclcpp::Time&, const rclcpp::Duration&) {
    uint8_t m = last_.load();
    bool ok_all = true;
    for (size_t i=0;i<command_interfaces_.size(); ++i) {
      double v = ((m >> i) & 0x1) ? 1.0 : 0.0;
      ok_all &= command_interfaces_[i].set_value(v);  // ← comprobar retorno
    }
    if (!ok_all) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "Algún set_value() falló en update()");
    }
    return controller_interface::return_type::OK;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(rpi5_gpio_control::RpiValveController, controller_interface::ControllerInterface)
