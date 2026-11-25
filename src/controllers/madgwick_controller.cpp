#include "sura_localization/controllers/madgwick_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace sura_localization::controllers
{

controller_interface::CallbackReturn MadgwickController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MadgwickController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
MadgwickController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn MadgwickController::on_configure(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MadgwickController::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MadgwickController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MadgwickController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  return controller_interface::return_type::OK;
}

}  // namespace sura_localization::controllers

PLUGINLIB_EXPORT_CLASS(
  sura_localization::controllers::MadgwickController,
  controller_interface::ControllerInterface)
