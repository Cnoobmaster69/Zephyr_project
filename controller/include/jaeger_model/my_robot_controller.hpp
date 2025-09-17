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
#include "std_msgs/msg/u_int32.hpp"

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
  double deadband_{0.1};

  // Subs (no-RT)
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;

  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::UInt32>> valves_mask_pub_;

  // Buffers RT
  realtime_tools::RealtimeBuffer<std::shared_ptr<nav_msgs::msg::Odometry>> odom_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Point>> target_buffer_;

  // Callbacks (no-RT)
  void odom_cb_(const nav_msgs::msg::Odometry::SharedPtr msg);
  void target_cb_(const geometry_msgs::msg::Point::SharedPtr msg);

  double kp_dist_{0.05};   // tunable: ganancia proporcional para control de posición
  double kp_yaw_{0.01};    // tunable: ganancia proporcional para control de orientación
  double v_max_{0.5};    // tunable: velocidad lineal máxima (m/s)
  double w_max_{2.0};    // tunable: velocidad angular máxima (rad/s)
  double base_half_width_{0.5};
  double hyst_band_{0.2}; // tunable: banda de histéresis para evitar oscilaciones ON/OFF
  double pwm_period_{0.1}; // tunable: periodo del PWM interno (segundos)}
  double pwm_phase_{0.0};  // fase del PWM interno (segundos)
  double deadband_pos_{0.2}; // tunable: zona muerta para considerar que se ha llegado a la posición (m)
  double deadband_yaw_{0.2};  // tunable: zona muerta para considerar que se ha llegado a la orientación (rad)

//     // Estado para el eje X
// enum class XMode { Idle, Forward, Backward };
// XMode x_mode_{XMode::Idle};

// // Anti-chatter
// double min_on_time_s_{0.20};   // tunable: 200 ms
// double since_switch_s_{0.0};

// // Histéresis
// double x_on_th_{0.3};   // entra a ON si |error| > 0.10 m (ejemplo)
// double x_off_th_{0.10};  // vuelve a IDLE si |error| < 0.05 m

};

}  // namespace jaeger_model
#endif  // JAEGER_MODEL__MY_ROBOT_CONTROLLER_HPP_
