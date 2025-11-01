#include "jaeger_model/my_robot_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <tf2/utils.hpp>                                 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>      



namespace jaeger_model
{

// ===============  Ciclo de vida  ===================

controller_interface::CallbackReturn MyRobotController::on_init()
{
  // get_node() está disponible en controllers; úsalo para declarar parámetros.
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotController"), "get_node() devolvió nullptr en on_init()");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Parámetros
  valve_names_ = node->declare_parameter<std::vector<std::string>>(
      "valve_names", std::vector<std::string>{"valve1","valve2","valve3","valve4"});

  odom_topic_     = node->declare_parameter<std::string>("odom_topic",     "odometry/filtered");
  setpoint_topic_ = node->declare_parameter<std::string>("setpoint_topic", "position_setpoint");
  deadband_       = node->declare_parameter<double>("deadband", 0.2);  // metros

  // Buffers RT inicializados con nullptr (no bloqueo)
  odom_buffer_.writeFromNonRT(std::shared_ptr<nav_msgs::msg::Odometry>(nullptr));
  target_buffer_.writeFromNonRT(std::shared_ptr<geometry_msgs::msg::Point>(nullptr));

  if (valve_names_.size() != 4) {
    RCLCPP_WARN(node->get_logger(),
      "Se esperaban 4 válvulas, pero 'valve_names' tiene %zu. El controlador funcionará con ese tamaño.",
      valve_names_.size());
  }
  RCLCPP_INFO(get_node()->get_logger(), "on_init()");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MyRobotController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotController"), "get_node() nullptr en on_configure()");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Suscripciones (NO en update(); aquí, fuera de tiempo real)
  using std::placeholders::_1;

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&MyRobotController::odom_cb_, this, _1));

  target_sub_ = node->create_subscription<geometry_msgs::msg::Point>(
      setpoint_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&MyRobotController::target_cb_, this, _1));

  valves_mask_pub_ = std::make_shared<
  realtime_tools::RealtimePublisher<std_msgs::msg::UInt32>>(
    get_node()->create_publisher<std_msgs::msg::UInt32>("valves_cmd_mask", 100));

  kp_rho_  = node->declare_parameter<double>("kp_rho",  kp_rho_);
  kd_vx_   = node->declare_parameter<double>("kd_vx",   kd_vx_);
  kp_yaw_  = node->declare_parameter<double>("kp_yaw",  kp_yaw_);
  kd_yaw_  = node->declare_parameter<double>("kd_yaw",  kd_yaw_);
  pos_deadband_   = node->declare_parameter<double>("pos_deadband", pos_deadband_);
  yaw_deadband_   = node->declare_parameter<double>("yaw_deadband", yaw_deadband_);
  vel_deadband_   = node->declare_parameter<double>("vel_deadband", vel_deadband_);
  duty_slew_rate_ = node->declare_parameter<double>("duty_slew_rate", duty_slew_rate_);
  pwm_hz_         = node->declare_parameter<double>("pwm_hz", pwm_hz_);
  pwm_period_     = 1.0 / std::max(1e-6, pwm_hz_); // recalcula por si cambia
  min_cos_for_surge_ = node->declare_parameter<double>("min_cos_for_surge", min_cos_for_surge_);

  RCLCPP_INFO(node->get_logger(),
      "MyRobotController configurado. odom_topic='%s', setpoint_topic='%s', deadband=%.3f",
      odom_topic_.c_str(), setpoint_topic_.c_str(), deadband_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MyRobotController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotController"), "get_node() nullptr en on_activate()");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (command_interfaces_.size() != valve_names_.size()) {
    RCLCPP_ERROR(node->get_logger(),
      "Número de interfaces de comando (%zu) != número de válvulas configuradas (%zu).",
      command_interfaces_.size(), valve_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Apaga todas las válvulas al activar
  for (auto &ci : command_interfaces_) {
    ci.set_value(0.0);
  }

  // Inicializa buffers con valores por defecto
  odom_buffer_.writeFromNonRT(std::make_shared<nav_msgs::msg::Odometry>());
  target_buffer_.writeFromNonRT(std::make_shared<geometry_msgs::msg::Point>());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MyRobotController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Apaga válvulas y suelta subs
  for (auto &ci : command_interfaces_) {
    ci.set_value(0.0);
  }
  odom_sub_.reset();
  target_sub_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

// ===============  Configuración de interfaces  ===================

controller_interface::InterfaceConfiguration
MyRobotController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Reclama explícitamente <valve>/digital para cada válvula
  cfg.names.reserve(valve_names_.size());
  for (const auto &v : valve_names_) {
    cfg.names.push_back(v + "/digital");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
MyRobotController::state_interface_configuration() const
{
  // Este controlador no necesita leer estados del hardware
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::NONE;
  return cfg;
}

// ===============  Lazo de control (tiempo real)  ===================

controller_interface::return_type
MyRobotController::update(const rclcpp::Time&, const rclcpp::Duration& period)
{
  const double dt = period.seconds();

  // 1) Leer odom y setpoint
  auto odom_ptr   = *odom_buffer_.readFromRT();
  auto target_ptr = *target_buffer_.readFromRT();
  if (!odom_ptr || !target_ptr) {
    for (auto &ci : command_interfaces_) { (void)ci.set_value(0.0); }
    if (valves_mask_pub_ && valves_mask_pub_->trylock()) {
      valves_mask_pub_->msg_.data = 0u;
      valves_mask_pub_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
  }

  double yaw_dir_= 1.0; // Sentido horario
  // ---- Estados (frame odom) y yaw
  const double x  = odom_ptr->pose.pose.position.x;
  const double y  = odom_ptr->pose.pose.position.y;

  double psi = tf2::getYaw(odom_ptr->pose.pose.orientation);  // yaw ROS (CCW+).

  // Guardas anti-paquete-nulo (opcional pero útil)
  auto &q = odom_ptr->pose.pose.orientation;
  const bool quat_ident = (std::abs(q.w) > 0.999 && std::abs(q.x)<1e-6 && std::abs(q.y)<1e-6 && std::abs(q.z)<1e-6);
  const bool pose_zero  = (std::abs(x)<1e-6 && std::abs(y)<1e-6);
  const bool invalid_sample = quat_ident && pose_zero;

  // Estima por diferencias: solo si tenemos previo y muestra válida y dt>0
  double vx = vx_est_, r = r_est_;
  if (!invalid_sample && have_prev_ && dt > 1e-6) {
  const double dx = x - prev_x_;
  const double dy = y - prev_y_;
  // wrap de delta yaw a [-pi,pi] para derivar correctamente
  const double dpsi = std::atan2(std::sin(psi - prev_psi_), std::cos(psi - prev_psi_)); 

  // velocidad en eje x del cuerpo (proyección de la velocidad espacial) 
  const double vx_inst = ( std::cos(psi)*dx + std::sin(psi)*dy ) / dt;
  const double r_inst  = dpsi / dt;

  // EMA para suavizar (evita dientes de sierra) 
  vx_est_ = (1.0 - ema_alpha_) * vx_est_ + ema_alpha_ * vx_inst;
  r_est_  = (1.0 - ema_alpha_) * r_est_  + ema_alpha_ * r_inst;

  vx = vx_est_;
  r  = r_est_;
  }

    // Actualiza memoria (si la muestra no es inválida)
  if (!invalid_sample) {
    prev_x_ = x; prev_y_ = y; prev_psi_ = psi; have_prev_ = true;
  }
  // const double vx = odom_ptr->twist.twist.linear.x;
  // double r  = odom_ptr->twist.twist.angular.z;
  r*= yaw_dir_; // Ajustar sentido pq no c cual ptas son los pines

  const double xd = target_ptr->x;
  const double yd = target_ptr->y;

  psi*= yaw_dir_; // Ajusta sentido
  const double psid = std::atan2(yd - y, xd - x);
  auto wrapPi = [](double a){ while (a>M_PI) a-=2*M_PI; while (a<-M_PI) a+=2*M_PI; return a; };
  const double epsi = wrapPi(psid - psi);
  const double rho  = std::hypot(xd - x, yd - y);

  // ---- PWM timing 
  pwm_phase_   += pwm_hz_ * dt;
  if (pwm_phase_ >= 1.0) pwm_phase_ -= std::floor(pwm_phase_);
  pwm_elapsed_ += dt;
  const bool recompute_duty = (pwm_elapsed_ >= pwm_period_);

  if (recompute_duty) {
    pwm_elapsed_ = std::fmod(pwm_elapsed_, pwm_period_);

    // 2) Lazos PD: yaw y avance (surge)
    //    Fwd pide más cuando está alineado (cos(epsi)); frena con vel. vx
    double Fx   =  kp_rho_ * rho * std::cos(epsi) - kd_vx_ * vx;
    double tauz =  kp_yaw_ * epsi - kd_yaw_ * r;

    // Opcional: reducir Fx si desalineado
    if (min_cos_for_surge_ > 0.0) {
      const double c = std::cos(epsi);
      if (c < min_cos_for_surge_) {
        Fx *= (c / (min_cos_for_surge_));
      }
    }

    // 3) Saturaciones físicas
    const double F_tot_max  = u_max_newton_;           // 20 N JSJAJAJ este mi error 
    const double F_side_max = 0.5 * F_tot_max;         // por lado
    const double tau_max    = half_track_m_ * F_tot_max;

    Fx   = std::clamp(Fx,   -F_tot_max, F_tot_max);
    tauz = std::clamp(tauz, -tau_max,   tau_max);
    
    if (rho < yaw_deadband_) tauz = 0.0;
    if (std::abs(epsi) < pos_deadband_) Fx = 0.0;

    // 4) Mezcla a lados (diferencial)
    double FL = 0.5 * (Fx + (tauz / std::max(1e-6, half_track_m_)));
    double FR = 0.5 * (Fx - (tauz / std::max(1e-6, half_track_m_)));

    // 5) Saturar por lado y mapear a duty por válvula
    FL = std::clamp(FL, -F_side_max, F_side_max);
    FR = std::clamp(FR, -F_side_max, F_side_max);

    auto dutyFromF = [&](double F)->std::pair<double,double> {
      // {duty_fwd, duty_rev} en [0,1]
      if (F >= 0.0) return { std::min(1.0, F / F_side_max), 0.0 };
      else          return { 0.0, std::min(1.0, (-F) / F_side_max) };
    };

    auto [dL_fwd, dL_rev] = dutyFromF(FL);
    auto [dR_fwd, dR_rev] = dutyFromF(FR);

    duty_L_fwd_ = dL_fwd; duty_L_rev_ = dL_rev;
    duty_R_fwd_ = dR_fwd; duty_R_rev_ = dR_rev;

    // Parada suave cerca del objetivo q no me sirve aaaaaaaaaaaaa
    if (rho < pos_deadband_ && std::abs(epsi) < yaw_deadband_ && std::abs(vx) < vel_deadband_) {
      duty_L_fwd_ = duty_L_rev_ = duty_R_fwd_ = duty_R_rev_ = 0.0;
    }
    // bool yaw_ok = (std::abs(epsi) < yaw_deadband_) && (std::abs(r) < yaw_rate_deadband_);
    // bool pos_ok = (rho < pos_deadband_);

    // if ((stop_require_position_ ? (yaw_ok && pos_ok) : yaw_ok)) {
    //   duty_L_fwd_ = duty_L_rev_ = duty_R_fwd_ = duty_R_rev_ = 0.0;
    // }
  }

  // 6) PWM instantáneo por válvula (fase común a 12 Hz)
  const bool on_L_fwd = (pwm_phase_ < duty_L_fwd_);
  const bool on_L_rev = (pwm_phase_ < duty_L_rev_);
  const bool on_R_fwd = (pwm_phase_ < duty_R_fwd_);
  const bool on_R_rev = (pwm_phase_ < duty_R_rev_);

  // Mapa: [0]=valve1(izq +X), [1]=valve2(izq -X), [2]=valve3(der +X), [3]=valve4(der -X)
  auto set = [&](std::size_t i, bool on){ if (command_interfaces_.size()>i) (void)command_interfaces_[i].set_value(on?1.0:0.0); };
  set(0, on_L_fwd);
  set(1, on_L_rev);
  set(2, on_R_fwd);
  set(3, on_R_rev);
  for (std::size_t i = 4; i < command_interfaces_.size(); ++i) { (void)command_interfaces_[i].set_value(0.0); }

  // 7) Publicar máscara uint32
  uint32_t mask = 0u;
  if (command_interfaces_.size() >= 1 && command_interfaces_[0].get_value() > 0.5) mask |= (1u << 0);
  if (command_interfaces_.size() >= 2 && command_interfaces_[1].get_value() > 0.5) mask |= (1u << 1);
  if (command_interfaces_.size() >= 3 && command_interfaces_[2].get_value() > 0.5) mask |= (1u << 2);
  if (command_interfaces_.size() >= 4 && command_interfaces_[3].get_value() > 0.5) mask |= (1u << 3);

  if (valves_mask_pub_ && valves_mask_pub_->trylock()) {
    valves_mask_pub_->msg_.data = mask;
    valves_mask_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}



// ===============  Callbacks (no-RT)  ===================

void MyRobotController::odom_cb_(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_buffer_.writeFromNonRT(msg);
}

void MyRobotController::target_cb_(const geometry_msgs::msg::Point::SharedPtr msg)
{
  target_buffer_.writeFromNonRT(msg);
}

}  // namespace jaeger_model

// Exporta el plugin
PLUGINLIB_EXPORT_CLASS(jaeger_model::MyRobotController, controller_interface::ControllerInterface)
