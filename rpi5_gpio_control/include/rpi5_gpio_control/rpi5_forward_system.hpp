#pragma once
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <vector>
#include <string>

namespace rpi5_gpio_control {

class RpiForwardSystem : public hardware_interface::SystemInterface {
public:
hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<double> cmd_, state_;     // 0.0 / 1.0
  std::vector<std::string> names_;      // "valves/valve{i}/level"
  // Publicador ROS para reenviar a tu nodo RPi.GPIO
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_;
  std::string out_topic_ {"/valves/bitmask"};
};

} // namespace
