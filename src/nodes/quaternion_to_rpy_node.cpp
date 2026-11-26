// Reads the orientation quaternion published by the MadgwickController and
// republishes it as roll/pitch/yaw (ZYX intrinsic convention, REP-103).
//
// Output: geometry_msgs/Vector3Stamped with x = roll, y = pitch, z = yaw.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace sura_localization
{

class QuaternionToRpyNode : public rclcpp::Node
{
public:
  explicit QuaternionToRpyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("quaternion_to_rpy", options)
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/bluerov/controller/madgwick_controller/imu");
    const auto output_topic = declare_parameter<std::string>("output_topic", "~/rpy");
    in_degrees_ = declare_parameter<bool>("in_degrees", false);

    publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      output_topic, rclcpp::SystemDefaultsQoS());

    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(*msg);});

    RCLCPP_INFO(
      get_logger(), "Converting '%s' -> '%s' (%s)",
      subscription_->get_topic_name(), publisher_->get_topic_name(),
      in_degrees_ ? "degrees" : "radians");
  }

private:
  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    double w = msg.orientation.w;
    double x = msg.orientation.x;
    double y = msg.orientation.y;
    double z = msg.orientation.z;

    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (!std::isfinite(norm) || norm < 1.0e-9) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received invalid quaternion");
      return;
    }
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;

    const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    const double pitch = std::asin(sin_pitch);
    const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    const double scale = in_degrees_ ? 180.0 / M_PI : 1.0;

    geometry_msgs::msg::Vector3Stamped out;
    out.header = msg.header;
    out.vector.x = roll * scale;
    out.vector.y = pitch * scale;
    out.vector.z = yaw * scale;
    publisher_->publish(out);
  }

  bool in_degrees_{false};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_;
};

}  // namespace sura_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sura_localization::QuaternionToRpyNode>());
  rclcpp::shutdown();
  return 0;
}
