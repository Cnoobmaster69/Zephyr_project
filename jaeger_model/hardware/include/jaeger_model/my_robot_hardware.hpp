// my_robot_hardware.hpp
#ifndef JAEGER_MODEL__MY_ROBOT_HARDWARE_HPP_
#define JAEGER_MODEL__MY_ROBOT_HARDWARE_HPP_

#pragma once
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <unordered_map>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace jaeger_model {

  class MyRobotHardware : public hardware_interface::SystemInterface
  {
  public:
    hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;
  
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
    hardware_interface::return_type read(
      const rclcpp::Time &, const rclcpp::Duration &) override;
  
    hardware_interface::return_type write(
      const rclcpp::Time &, const rclcpp::Duration &) override;
  
  private:
    // Storage for all state interfaces declared under <joint> in <ros2_control>
    std::vector<std::pair<std::string, std::string>> joint_state_index_; // (joint_name, iface)
    std::vector<double> joint_state_storage_;
  
    // Storage for all command interfaces declared under <gpio> in <ros2_control>
    std::vector<std::pair<std::string, std::string>> gpio_cmd_index_; // (gpio_name, iface)
    std::vector<double> gpio_cmd_storage_;
  };
  
  } // namespace jaeger_model

#endif  // JAEGER_MODEL__MY_ROBOT_HARDWARE_HPP_