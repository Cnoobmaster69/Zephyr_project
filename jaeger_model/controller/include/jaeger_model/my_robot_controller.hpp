#ifndef JAEGER_MODEL__MY_ROBOT_CONTROLLER_HPP_
#define JAEGER_MODEL__MY_ROBOT_CONTROLLER_HPP_


#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace jaeger_model
{

class MyRobotController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & prev) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & prev) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & prev) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time & time,
                                           const rclcpp::Duration & period) override;

private:
  // Parámetros / configuración
  std::vector<std::string> valve_names_;
  std::string odom_topic_;
  std::string setpoint_topic_;
  double deadband_{0.01};

  // Subs (no-RT)
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;

  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> valves_pub_;

  // Buffers RT
  realtime_tools::RealtimeBuffer<std::shared_ptr<nav_msgs::msg::Odometry>> odom_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Point>> target_buffer_;

  // Callbacks (no-RT)
  void odom_cb_(const nav_msgs::msg::Odometry::SharedPtr msg);
  void target_cb_(const geometry_msgs::msg::Point::SharedPtr msg);
};

}  // namespace jaeger_model
#endif  // JAEGER_MODEL__MY_ROBOT_CONTROLLER_HPP_