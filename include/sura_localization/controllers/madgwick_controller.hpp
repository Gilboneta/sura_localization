#pragma once

#include <cstddef>
#include <string>

#include "controller_interface/controller_interface.hpp"

namespace sura_localization::controllers
{

class MadgwickController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct StateInterfaceIndices
  {
    std::size_t gyro_x{};
    std::size_t gyro_y{};
    std::size_t gyro_z{};
    std::size_t acceleration_x{};
    std::size_t acceleration_y{};
    std::size_t acceleration_z{};
    std::size_t imu_sample_time_sec{};
    std::size_t imu_sample_time_nanosec{};
    std::size_t magnetic_field_x{};
    std::size_t magnetic_field_y{};
    std::size_t magnetic_field_z{};
    std::size_t magnetometer_sample_time_sec{};
    std::size_t magnetometer_sample_time_nanosec{};
  };

  bool map_state_interfaces();

  std::string imu_sensor_name_;
  std::string magnetometer_sensor_name_;
  StateInterfaceIndices state_interface_indices_;
  bool state_interfaces_mapped_{false};

  double gyro_x_{0.0};
  double gyro_y_{0.0};
  double gyro_z_{0.0};
  double acceleration_x_{0.0};
  double acceleration_y_{0.0};
  double acceleration_z_{0.0};
  double imu_sample_time_sec_{0.0};
  double imu_sample_time_nanosec_{0.0};
  double magnetic_field_x_{0.0};
  double magnetic_field_y_{0.0};
  double magnetic_field_z_{0.0};
  double magnetometer_sample_time_sec_{0.0};
  double magnetometer_sample_time_nanosec_{0.0};
};

}  // namespace sura_localization::controllers
