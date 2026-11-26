#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sura_localization/controllers/madgwick_filter.hpp"

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

  using ImuMsg = sensor_msgs::msg::Imu;
  using ImuPublisher = realtime_tools::RealtimePublisher<ImuMsg>;

  bool map_state_interfaces();

  void publish_orientation();

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

  MadgwickFilter filter_;
  double max_imu_dt_{0.5};
  double last_imu_sample_time_{0.0};
  bool has_last_imu_sample_time_{false};

  bool use_magnetometer_{true};
  double max_magnetometer_age_{0.1};

  std::string frame_id_;
  double orientation_stddev_{0.02};
  rclcpp::Publisher<ImuMsg>::SharedPtr imu_publisher_;
  std::unique_ptr<ImuPublisher> realtime_imu_publisher_;
};

}  // namespace sura_localization::controllers
