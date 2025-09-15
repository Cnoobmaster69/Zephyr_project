#pragma once
#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace rpi5_gpio_control {

class RpiValveController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
  void cb(const std_msgs::msg::UInt8 & msg);
  std::vector<std::string> interfaces_;
  std::string topic_{"/valves/bitmask_cmd"};
  std::atomic<uint8_t> last_{0};
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_;
};

} // namespace
