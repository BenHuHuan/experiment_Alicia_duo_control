#include "alicia_duo_ros_control/alicia_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alicia_duo_ros_control
{
hardware_interface::CallbackReturn AliciaHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  size_t n_joints = info.joints.size();
  hw_positions_.resize(n_joints, 0.0);
  hw_velocities_.resize(n_joints, 0.0);
  hw_commands_.resize(n_joints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AliciaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AliciaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type AliciaHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // 示例模拟读取，实际你要从硬件读取
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AliciaHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // 示例模拟写入，实际你要写入硬件
  return hardware_interface::return_type::OK;
}
}  // namespace alicia_duo_ros_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(alicia_duo_ros_control::AliciaHardwareInterface, hardware_interface::SystemInterface)
