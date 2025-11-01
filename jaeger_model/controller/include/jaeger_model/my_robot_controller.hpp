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
  
  
    // // Ganancias y límites
    // double kp_x_{0.3};
    // double kd_x_{1.0};
    // double u_max_newton_{20.0};   // saturación de fuerza ±20 N
  
    // // PWM
    // double pwm_hz_{12.0};
    // double pwm_period_{1.0/12.0};
    // double pwm_phase_{0.0};       // [0,1) diente de sierra
    // double pwm_elapsed_{0.0};     // acumulador para recalcular duty
    // double duty_fwd_{0.0};
    // double duty_rev_{0.0};
  
    // // Bandas muertas
    // double pos_deadband_{0.01};   // ya usabas deadband_; puedes unificar
    // double vel_deadband_{0.01};   // opcional para evitar “creep”
  
  // --- Ganancias (a tunear) ---
  double kp_rho_{0.0};     // [N/m]   empuje vs distancia al objetivo
  double kd_vx_{0.0};      // [N·s/m] freno por velocidad en eje x del robot
  double kp_yaw_{0.0};     // [N·m/rad] par de guiñada vs error de yaw
  double kd_yaw_{0.0};     // [N·m·s/rad] freno por velocidad angular (r)
  
  // --- Geometría / límites físicos ---
  double half_track_m_{0.20};   // [m] semidistancia lateral (brazo b)
  double u_max_newton_{0.5};   // [N] empuje total máx (2 válvulas en paralelo)
                                // => por lado: F_side_max = u_max/2
  // Límite de par máximo implícito: tau_max = b * u_max
  
  // --- Bandas muertas / gating ---
  double pos_deadband_{0.1};       // [m] cerca del objetivo, apaga
  double yaw_deadband_{5.0 * M_PI/180.0}; // [rad] ~2°
  double vel_deadband_{0.03};       // [m/s] evita creep
  
  // Opcional: reducir avance si hay gran desalineación (0 desactiva)
  double min_cos_for_surge_{0.0};   // [0..1], p.ej. 0.2 si quieres gatear surge
  
  // --- PWM (si no los tienes ya) ---
  double pwm_hz_{12.0};             // [Hz] frecuencia del PWM “soft”
  double pwm_period_{1.0/12.0};     // [s]  período
  double pwm_phase_{0.0};
  double pwm_elapsed_{0.0};
  
  // --- Duties por válvula (por lado) ---
  double duty_L_fwd_{0.0};  // válvula 1 (izq +X)
  double duty_L_rev_{0.0};  // válvula 2 (izq -X)
  double duty_R_fwd_{0.0};  // válvula 3 (der +X)
  double duty_R_rev_{0.0};  // válvula 4 (der -X)
  
  // --- (Opcional) Suavizado de duty entre periodos para menos “clack” ---
  double duty_slew_rate_{0.05}; // cambio máx de duty por recomputación (0..1)
  
  // --- Helpers/estado interno (si no existen) ---
  inline double wrapPi(double a) const {
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }
  
  double yaw_rate_deadband_{0.05};   // [rad/s]
  bool   stop_require_position_{true}; // en pruebas de yaw: false
  
  // Estimadores por diferencias finitas
  bool have_prev_{false};
  double prev_x_{0.0}, prev_y_{0.0}, prev_psi_{0.0};
  double vx_est_{0.0}, r_est_{0.0};      // estimados suavizados
  double ema_alpha_{0.3};                // 0..1 (más alto = más reactivo)

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