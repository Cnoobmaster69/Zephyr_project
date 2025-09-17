
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
    get_node()->create_publisher<std_msgs::msg::UInt32>("valves_cmd_mask", 25));
    

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
MyRobotController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Lee último odom y setpoint desde buffers RT (no bloquea)
  auto odom_ptr   = *odom_buffer_.readFromRT();
  auto target_ptr = *target_buffer_.readFromRT();

  if (!odom_ptr || !target_ptr) {
    // Aún no hay datos; mantener todo apagado
    for (auto &ci : command_interfaces_) {
      ci.set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // 2) Errores geométricos (plano XY)
  const double x  = odom_ptr->pose.pose.position.x;
  const double y  = odom_ptr->pose.pose.position.y;
  const auto& q   = odom_ptr->pose.pose.orientation;
  const double yaw = std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));

  const double xd = target_ptr->x, yd = target_ptr->y;
  const double dx = xd - x,    dy = yd - y;
  const double dist = std::hypot(dx, dy);
  const double yaw_ref = std::atan2(dy, dx);
  double eyaw = std::atan2(std::sin(yaw_ref - yaw), std::cos(yaw_ref - yaw));

  // 3) Control continuo sencillo (guiado)
  //    tune: kp_dist, kp_yaw, v_max, w_max, deadband
  double v = kp_dist_ * dist;
  double w = kp_yaw_  * eyaw;

  // bandas muertas para no vibrar cerca del objetivo / rumbo
  if (dist < deadband_pos_) v = 0.0;
  if (std::abs(eyaw) < deadband_yaw_) w = 0.0;

  // saturación
  v = std::clamp(v, -v_max_, v_max_);
  w = std::clamp(w, -w_max_, w_max_);

  // 4) Mapeo a lados (diferencial) y duty PWM
  //    b = semidistancia entre lados (parámetro geométrico simple)
  double uL = std::clamp((v - w * base_half_width_) / v_max_, -1.0, 1.0);
  double uR = std::clamp((v + w * base_half_width_) / v_max_, -1.0, 1.0);

  // Histeresis para evitar oscilaciones ON/OFF cerca de 0
  auto with_hyst = [this](double u) {
    if (std::abs(u) < hyst_band_) return 0.0;
    return u;
  };
  uL = with_hyst(uL);  uR = with_hyst(uR);

  // PWM interno (a 10 Hz por ejemplo)
  pwm_phase_ += period.seconds();
  if (pwm_phase_ >= pwm_period_) pwm_phase_ -= pwm_period_;
  auto pwm_on = [this](double duty){ duty = std::clamp(duty, 0.0, 1.0); return (pwm_phase_ < duty * pwm_period_); };

  // 5) Decisión válvulas ON/OFF por lado
  const bool L_fwd = (uL > 0.0) && pwm_on(std::abs(uL));
  const bool L_rev = (uL < 0.0) && pwm_on(std::abs(uL));
  const bool R_fwd = (uR > 0.0) && pwm_on(std::abs(uR));
  const bool R_rev = (uR < 0.0) && pwm_on(std::abs(uR));

  // Asignación: [0]=V1(+X/L_fwd), [1]=V2(-X/L_rev), [2]=V3(+Y/R_fwd), [3]=V4(-Y/R_rev)
  const std::size_t n = command_interfaces_.size();
  if (n >= 1) command_interfaces_[0].set_value(L_fwd ? 1.0 : 0.0);
  if (n >= 2) command_interfaces_[1].set_value(L_rev ? 1.0 : 0.0);
  if (n >= 3) command_interfaces_[2].set_value(R_fwd ? 1.0 : 0.0);
  if (n >= 4) command_interfaces_[3].set_value(R_rev ? 1.0 : 0.0);
  for (std::size_t i = 4; i < n; ++i) command_interfaces_[i].set_value(0.0);

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
