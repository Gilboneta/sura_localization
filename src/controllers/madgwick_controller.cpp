#include "sura_localization/controllers/madgwick_controller.hpp"

#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace sura_localization::controllers
{

controller_interface::CallbackReturn MadgwickController::on_init()
{
  try {
    auto_declare<std::string>("imu_sensor_name", "imu_sensor");
    auto_declare<std::string>("magnetometer_sensor_name", "magnetometer_sensor");
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to declare MadgwickController parameters: %s",
      exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }

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
  const auto imu_sensor_name = get_node()->get_parameter("imu_sensor_name").as_string();
  const auto magnetometer_sensor_name =
    get_node()->get_parameter("magnetometer_sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      imu_sensor_name + "/angular_velocity.x",
      imu_sensor_name + "/angular_velocity.y",
      imu_sensor_name + "/angular_velocity.z",
      imu_sensor_name + "/linear_acceleration.x",
      imu_sensor_name + "/linear_acceleration.y",
      imu_sensor_name + "/linear_acceleration.z",
      imu_sensor_name + "/sample_time.sec",
      imu_sensor_name + "/sample_time.nanosec",
      magnetometer_sensor_name + "/magnetic_field.x",
      magnetometer_sensor_name + "/magnetic_field.y",
      magnetometer_sensor_name + "/magnetic_field.z",
      magnetometer_sensor_name + "/sample_time.sec",
      magnetometer_sensor_name + "/sample_time.nanosec",
    }};
}

controller_interface::CallbackReturn MadgwickController::on_configure(
  const rclcpp_lifecycle::State &)
{
  imu_sensor_name_ = get_node()->get_parameter("imu_sensor_name").as_string();
  magnetometer_sensor_name_ =
    get_node()->get_parameter("magnetometer_sensor_name").as_string();

  if (imu_sensor_name_.empty() || magnetometer_sensor_name_.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "imu_sensor_name and magnetometer_sensor_name must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  state_interfaces_mapped_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MadgwickController::on_activate(
  const rclcpp_lifecycle::State &)
{
  state_interfaces_mapped_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MadgwickController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  state_interfaces_mapped_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MadgwickController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!state_interfaces_mapped_ && !map_state_interfaces()) {
    return controller_interface::return_type::ERROR;
  }

  const auto read_interface = [this](const std::size_t index) {
      return state_interfaces_[index].get_value();
    };

  gyro_x_ = read_interface(state_interface_indices_.gyro_x);
  gyro_y_ = read_interface(state_interface_indices_.gyro_y);
  gyro_z_ = read_interface(state_interface_indices_.gyro_z);

  acceleration_x_ = read_interface(state_ion_configurenterface_indices_.acceleration_x);
  acceleration_y_ = read_interface(state_interface_indices_.acceleration_y);
  acceleration_z_ = read_interface(state_interface_indices_.acceleration_z);

  imu_sample_time_sec_ = read_interface(state_interface_indices_.imu_sample_time_sec);
  imu_sample_time_nanosec_ =
    read_interface(state_interface_indices_.imu_sample_time_nanosec);

  magnetic_field_x_ = read_interface(state_interface_indices_.magnetic_field_x);
  magnetic_field_y_ = read_interface(state_interface_indices_.magnetic_field_y);
  magnetic_field_z_ = read_interface(state_interface_indices_.magnetic_field_z);

  magnetometer_sample_time_sec_ =
    read_interface(state_interface_indices_.magnetometer_sample_time_sec);
  magnetometer_sample_time_nanosec_ =
    read_interface(state_interface_indices_.magnetometer_sample_time_nanosec);

  const double values[] = {
    gyro_x_, gyro_y_, gyro_z_,
    acceleration_x_, acceleration_y_, acceleration_z_,
    imu_sample_time_sec_, imu_sample_time_nanosec_,
    magnetic_field_x_, magnetic_field_y_, magnetic_field_z_,
    magnetometer_sample_time_sec_, magnetometer_sample_time_nanosec_};

  for (const double value : values) {
    if (!std::isfinite(value)) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "MadgwickController received a non-finite sensor interface value");
      return controller_interface::return_type::ERROR;
    }
  }

  // The Madgwick filter will consume these values in a later commit.
  return controller_interface::return_type::OK;
}

bool MadgwickController::map_state_interfaces()
{
  const std::vector<std::string> expected_names{
    imu_sensor_name_ + "/angular_velocity.x",
    imu_sensor_name_ + "/angular_velocity.y",
    imu_sensor_name_ + "/angular_velocity.z",
    imu_sensor_name_ + "/linear_acceleration.x",
    imu_sensor_name_ + "/linear_acceleration.y",
    imu_sensor_name_ + "/linear_acceleration.z",
    imu_sensor_name_ + "/sample_time.sec",
    imu_sensor_name_ + "/sample_time.nanosec",
    magnetometer_sensor_name_ + "/magnetic_field.x",
    magnetometer_sensor_name_ + "/magnetic_field.y",
    magnetometer_sensor_name_ + "/magnetic_field.z",
    magnetometer_sensor_name_ + "/sample_time.sec",
    magnetometer_sensor_name_ + "/sample_time.nanosec",
  };

  if (state_interfaces_.size() != expected_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, but received %zu",
      expected_names.size(), state_interfaces_.size());
    return false;
  }

  std::vector<std::size_t> indices(expected_names.size());
  for (std::size_t expected_index = 0; expected_index < expected_names.size(); ++expected_index) {
    bool found = false;
    for (std::size_t actual_index = 0; actual_index < state_interfaces_.size(); ++actual_index) {
      if (state_interfaces_[actual_index].get_name() == expected_names[expected_index]) {
        indices[expected_index] = actual_index;
        found = true;
        break;
      }
    }

    if (!found) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Required state interface '%s' was not assigned",
        expected_names[expected_index].c_str());
      return false;
    }
  }

  state_interface_indices_.gyro_x = indices[0];
  state_interface_indices_.gyro_y = indices[1];
  state_interface_indices_.gyro_z = indices[2];
  state_interface_indices_.acceleration_x = indices[3];
  state_interface_indices_.acceleration_y = indices[4];
  state_interface_indices_.acceleration_z = indices[5];
  state_interface_indices_.imu_sample_time_sec = indices[6];
  state_interface_indices_.imu_sample_time_nanosec = indices[7];
  state_interface_indices_.magnetic_field_x = indices[8];
  state_interface_indices_.magnetic_field_y = indices[9];
  state_interface_indices_.magnetic_field_z = indices[10];
  state_interface_indices_.magnetometer_sample_time_sec = indices[11];
  state_interface_indices_.magnetometer_sample_time_nanosec = indices[12];
  state_interfaces_mapped_ = true;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "MadgwickController mapped IMU and magnetometer state interfaces");
  return true;
}

}  // namespace sura_localization::controllers

PLUGINLIB_EXPORT_CLASS(
  sura_localization::controllers::MadgwickController,
  controller_interface::ControllerInterface)
