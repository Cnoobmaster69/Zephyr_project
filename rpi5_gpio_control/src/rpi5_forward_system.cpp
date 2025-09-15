#include "rpi5_gpio_control/rpi5_forward_system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rpi5_gpio_control {

hardware_interface::CallbackReturn
RpiForwardSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
  {
    // accede al HardwareInfo así:
    const auto & info = params.hardware_info;
  
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
  

  // 5 interfaces por defecto; opcionalmente leer "count" del URDF
  size_t n = 5;
  auto it = info.hardware_parameters.find("count");
  if (it != info.hardware_parameters.end()) n = std::stoul(it->second);

  cmd_.assign(n, 0.0);
  state_.assign(n, 0.0);

  names_.clear();
  for (size_t i=0;i<n;++i)
    names_.push_back("valves/valve" + std::to_string(i) + "/level");

  auto it_topic = info.hardware_parameters.find("out_topic");
  if (it_topic != info.hardware_parameters.end()) out_topic_ = it_topic->second;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
RpiForwardSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Crea un node interno para publicar el bitmask
  node_ = std::make_shared<rclcpp::Node>("rpi5_forward_hw");
  pub_  = node_->create_publisher<std_msgs::msg::UInt8>(out_topic_, 10);
  // Inicializa a 0
  for (auto & c : cmd_)   c = 0.0;
  for (auto & s : state_) s = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
RpiForwardSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Apaga todo (publica 0) y limpia publisher
  if (pub_) {
    std_msgs::msg::UInt8 msg; msg.data = 0;
    pub_->publish(msg);
  }
  pub_.reset(); node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RpiForwardSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifs;
  ifs.reserve(state_.size());
  for (size_t i=0;i<state_.size();++i)
    ifs.emplace_back(hardware_interface::StateInterface(names_[i],
        hardware_interface::HW_IF_POSITION, &state_[i]));
  return ifs;
}

std::vector<hardware_interface::CommandInterface>
RpiForwardSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifs;
  ifs.reserve(cmd_.size());
  for (size_t i=0;i<cmd_.size();++i)
    ifs.emplace_back(hardware_interface::CommandInterface(names_[i],
        hardware_interface::HW_IF_POSITION, &cmd_[i]));
  return ifs;
}

hardware_interface::return_type
RpiForwardSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Refleja el último comando como estado (no hay HW real aquí)
  state_ = cmd_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
RpiForwardSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Empaqueta command_interfaces_ en un bitmask y publícalo
  if (!pub_) return hardware_interface::return_type::OK;
  uint8_t mask = 0;
  for (size_t i=0;i<cmd_.size() && i<8; ++i)
    if (cmd_[i] > 0.5) mask |= (1u << i);

  std_msgs::msg::UInt8 msg; msg.data = mask;
  pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(rpi5_gpio_control::RpiForwardSystem, hardware_interface::SystemInterface)
