#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace
{

class PressureToPose : public rclcpp::Node
{
public:
  PressureToPose()
  : Node("pressure_to_pose")
  {
    declare_parameter<std::string>("input_topic", "sensors/pressure");
    declare_parameter<std::string>("output_topic", "sensors/pressure/pose_enu");
    declare_parameter<std::string>("environment", "real");
    declare_parameter<std::string>("frame_id", "world_enu");
    declare_parameter<std::string>("sensor_frame_id", "");
    declare_parameter<bool>("positive_down", true);
    declare_parameter<double>("surface_pressure_pa", 101325.0);
    declare_parameter<double>("fluid_density_kg_m3", 997.0);
    declare_parameter<double>("gravity_m_s2", 9.80665);
    declare_parameter<double>("z_scale", 1.0);
    declare_parameter<double>("z_offset_m", 0.0);
    declare_parameter<double>("fallback_z_variance", 0.01);
    declare_parameter<double>("fallback_xy_variance", 99999.0);

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();

    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic,
      10);
    subscription_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      input_topic,
      10,
      [this](const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
        on_pressure(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Converting pressure %s to pose %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  void on_pressure(const sensor_msgs::msg::FluidPressure & msg)
  {
    const double density = get_parameter("fluid_density_kg_m3").as_double();
    const double gravity = get_parameter("gravity_m_s2").as_double();
    if (density <= 0.0 || gravity <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Invalid density or gravity; skipping pressure sample");
      return;
    }

    const auto environment = get_parameter("environment").as_string();
    double depth = 0.0;

    if (environment == "sim") {
      const double pressure_delta_pa = msg.fluid_pressure;
      depth = pressure_delta_pa / (density * gravity);
    } else {
      const double surface_pressure = get_parameter("surface_pressure_pa").as_double();
      const double pressure_delta_pa = msg.fluid_pressure - surface_pressure;
      depth = pressure_delta_pa / (density * gravity);
    }

    if (!std::isfinite(depth)) {
      return;
    }

    const bool positive_down = get_parameter("positive_down").as_bool();
    const double raw_z = positive_down ? -depth : depth;
    const double z = raw_z * get_parameter("z_scale").as_double() +
      get_parameter("z_offset_m").as_double();

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = msg.header.stamp;
    pose.header.frame_id = get_parameter("frame_id").as_string();
    pose.pose.pose.position.z = z;
    pose.pose.pose.orientation.w = 1.0;

    double z_variance = get_parameter("fallback_z_variance").as_double();
    if (msg.variance > 0.0 && std::isfinite(msg.variance)) {
      z_variance = msg.variance / ((density * gravity) * (density * gravity));
    }

    const double xy_variance = get_parameter("fallback_xy_variance").as_double();
    pose.pose.covariance[0] = std::max(0.0, xy_variance);
    pose.pose.covariance[7] = std::max(0.0, xy_variance);
    pose.pose.covariance[14] = std::max(0.0, z_variance);
    pose.pose.covariance[21] = 99999.0;
    pose.pose.covariance[28] = 99999.0;
    pose.pose.covariance[35] = 99999.0;

    publisher_->publish(pose);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscription_;

  bool has_sim_surface_pressure_{false};
  double sim_surface_pressure_{0.0};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PressureToPose>());
  rclcpp::shutdown();
  return 0;
}