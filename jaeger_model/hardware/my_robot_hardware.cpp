#include "jaeger_model/my_robot_hardware.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jaeger_model
{

hardware_interface::CallbackReturn MyRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Base init: parse <ros2_control> of the URDF
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Base on_init failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ====== Prepare storage for JOINT STATE interfaces (optional) ======
  // For every <joint> listed in <ros2_control> with <state_interface .../>
  // we allocate a double value initialized a 0.0 and remember its (joint,interface) order.
  joint_state_storage_.clear();
  joint_state_index_.clear();

  for (const auto & j : info_.joints)
  {
    for (const auto & si : j.state_interfaces)
    {
      joint_state_index_.push_back({j.name, si.name});
      joint_state_storage_.push_back(0.0);  // default
    }
  }

  // ====== Prepare storage for GPIO COMMAND interfaces (valves) ======
  // For every <gpio> listed with <command_interface name="digital"/>
  // we allocate a double command value (0/1).
  gpio_cmd_storage_.clear();
  gpio_cmd_index_.clear();

  for (const auto & g : info_.gpios)
  {
    for (const auto & ci : g.command_interfaces)
    {
      // we expect "digital" but we don't hardcode it in case you add more later
      gpio_cmd_index_.push_back({g.name, ci.name});
      gpio_cmd_storage_.push_back(0.0);  // OFF by default
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MyRobotHardware"),
    "MyRobotHardware on_init: %zu joint-state handles, %zu gpio-command handles",
    joint_state_storage_.size(), gpio_cmd_storage_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyRobotHardware::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    // Constructor C++ v1.x: (device, how). Usa OPEN_LOOKUP para buscar por nombre "gpiochipN".
    chip_ = gpiod::chip(chip_name_, gpiod::chip::OPEN_LOOKUP);

    lines_.clear();
    lines_.reserve(offsets_.size());

    const auto flags = active_low_
    ? gpiod::line_request::FLAG_ACTIVE_LOW
    : std::bitset<32>{};

    for (auto off : offsets_) {
      gpiod::line ln = chip_.get_line(off);                   // ← obtiene la línea
      ln.request({ "jaeger_hw", gpiod::line_request::DIRECTION_OUTPUT, flags }, 0);                      // ← salida nivel bajo
      lines_.push_back(std::move(ln));
    }

    // Failsafe: todo apagado
    for (auto &ln : lines_) ln.set_value(0);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"),
                 "GPIO configure failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(joint_state_storage_.size());

  // Export all state interfaces we discovered in on_init()
  for (size_t i = 0; i < joint_state_storage_.size(); ++i)
  {
    const auto & name = joint_state_index_[i].first;     // joint name
    const auto & iface = joint_state_index_[i].second;   // e.g. "position"
    out.emplace_back(hardware_interface::StateInterface(name, iface, &joint_state_storage_[i]));
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MyRobotHardware"),
    "Exported %zu state interfaces", out.size());
  return out;
}

std::vector<hardware_interface::CommandInterface> MyRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(gpio_cmd_storage_.size());

  // Export all GPIO command interfaces (valves)
  for (size_t i = 0; i < gpio_cmd_storage_.size(); ++i)
  {
    const auto & name = gpio_cmd_index_[i].first;      // gpio name, e.g. "valve1"
    const auto & iface = gpio_cmd_index_[i].second;    // e.g. "digital"
    out.emplace_back(hardware_interface::CommandInterface(name, iface, &gpio_cmd_storage_[i]));
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MyRobotHardware"),
    "Exported %zu GPIO command interfaces", out.size());
  return out;
}

hardware_interface::return_type MyRobotHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // No ROS subscriptions here:
  // If you kept x_joint/y_joint as state interfaces, they will stay with whatever value
  // you decide to keep here. For now, we don't change them (remain last value / 0.0).
  //
  // In simul/real HW you would read sensors here and update joint_state_storage_[i].

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyRobotHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Actuate GPIO lines according to command values
  const size_t n = std::min(lines_.size(), gpio_cmd_storage_.size());
  for (size_t i = 0; i < n; ++i) {
    const int v = (gpio_cmd_storage_[i] > 0.5) ? 1 : 0;  // 1=ON, 0=OFF
    lines_[i].set_value(v);
  }

  // Here you would actuate your real/virtual hardware using the latest commands.
  // For now we just log the current command values (0.0 ~ OFF, 1.0 ~ ON).
  if (!gpio_cmd_storage_.empty())
  {
    // throttle logging a little by printing every cycle is too chatty
    static uint32_t counter = 0;
    if ((counter++ % 50) == 0)
    {
      std::string s = "Valve commands: ";
      for (size_t i = 0; i < gpio_cmd_storage_.size(); ++i)
      {
        s += gpio_cmd_index_[i].first + "/" + gpio_cmd_index_[i].second + "="
          + std::to_string(gpio_cmd_storage_[i]) + (i + 1 < gpio_cmd_storage_.size() ? ", " : "");
      }
      RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "%s", s.c_str());
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MyRobotHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Apaga y libera (se liberan al destruir 'line', aquí sólo limpias)
  for (auto &ln : lines_) { if (ln) ln.set_value(0); }
  lines_.clear();
  return CallbackReturn::SUCCESS;
}

}  // namespace jaeger_model

// Register as a plugin
PLUGINLIB_EXPORT_CLASS(
  jaeger_model::MyRobotHardware,
  hardware_interface::SystemInterface
)
