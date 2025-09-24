#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <gpiod.hpp>                               // C++ bindings (libgpiodcxx 1.x)
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class GpioLinkNode : public rclcpp::Node
{
public:
  GpioLinkNode()
  : rclcpp::Node("gpiod_link_node")
  {
    // Parámetros (puedes cambiarlos por CLI/YAML)
    chip_name_ = declare_parameter<std::string>("gpio.chip", "gpiochip4");
    offset_    = declare_parameter<int>("gpio.offset", 17);          // BCM 17
    period_s_  = declare_parameter<double>("period_s", 1.0);         // 1 s

    // Abrir chip y solicitar línea como salida
    try {
      chip_ = std::make_unique<gpiod::chip>(chip_name_);
      line_ = std::make_unique<gpiod::line>(chip_->get_line(static_cast<unsigned>(offset_)));
      line_->request({ "gpiod_link_node",
                       gpiod::line_request::DIRECTION_OUTPUT,
                       0 /*flags*/ },
                      0 /*initial value*/);
      RCLCPP_INFO(get_logger(), "GPIO listo: %s offset %d", chip_name_.c_str(), offset_);
    } catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "Error al abrir/solicitar GPIO: %s", e.what());
      throw; // deja fallar si no se puede configurar el GPIO
    }

    // Timer para alternar el pin
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(period_s_));
    timer_ = create_wall_timer(ms, std::bind(&GpioLinkNode::tick, this));
  }

  ~GpioLinkNode() override
  {
    // Apaga el pin al salir (best-effort)
    try {
      if (line_ && line_->is_requested()) line_->set_value(0);
    } catch (...) {}
  }

private:
  void tick()
  {
    state_ = !state_;
    try {
      line_->set_value(state_ ? 1 : 0);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Blink %s:%d -> %d", chip_name_.c_str(), offset_, state_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "set_value() falló: %s", e.what());
    }
  }

  // Parámetros/estado
  std::string chip_name_;
  int offset_{17};
  double period_s_{1.0};
  bool state_{false};

  // Recursos libgpiod
  std::unique_ptr<gpiod::chip> chip_;
  std::unique_ptr<gpiod::line> line_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GpioLinkNode>());
  } catch (const std::exception &e) {
    // Ya se logueó; asegura cierre limpio de ROS
    (void)e;
  }
  rclcpp::shutdown();
  return 0;
}
