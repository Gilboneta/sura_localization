#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace
{

geometry_msgs::msg::Quaternion multiply_quaternions(
  const geometry_msgs::msg::Quaternion & left,
  const geometry_msgs::msg::Quaternion & right)
{
  geometry_msgs::msg::Quaternion result;
  result.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z;
  result.x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y;
  result.y = left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x;
  result.z = left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w;
  return result;
}

geometry_msgs::msg::Quaternion normalize_quaternion(geometry_msgs::msg::Quaternion quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x +
    quaternion.y * quaternion.y +
    quaternion.z * quaternion.z +
    quaternion.w * quaternion.w);
  if (norm <= 0.0) {
    geometry_msgs::msg::Quaternion identity;
    identity.w = 1.0;
    return identity;
  }
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return quaternion;
}

geometry_msgs::msg::Quaternion rotate_orientation(
  const geometry_msgs::msg::Quaternion & orientation)
{
  geometry_msgs::msg::Quaternion enu_to_ned;
  enu_to_ned.x = std::sqrt(0.5);
  enu_to_ned.y = std::sqrt(0.5);
  enu_to_ned.z = 0.0;
  enu_to_ned.w = 0.0;
  return normalize_quaternion(multiply_quaternions(enu_to_ned, orientation));
}

std::array<double, 36> transform_pose_covariance(const std::array<double, 36> & covariance)
{
  constexpr double transform[6][6] = {
    {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, -1.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, -1.0},
  };

  std::array<double, 36> converted{};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      double value = 0.0;
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          value += transform[row][i] * covariance[i * 6 + j] * transform[col][j];
        }
      }
      converted[row * 6 + col] = value;
    }
  }
  return converted;
}

std::array<double, 36> transform_twist_angular_covariance(
  const std::array<double, 36> & covariance)
{
  constexpr double transform[6][6] = {
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, -1.0},
  };

  std::array<double, 36> converted{};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      double value = 0.0;
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          value += transform[row][i] * covariance[i * 6 + j] * transform[col][j];
        }
      }
      converted[row * 6 + col] = value;
    }
  }
  return converted;
}

class EnuToNedOdometry : public rclcpp::Node
{
public:
  EnuToNedOdometry()
  : Node("enu_to_ned_odometry")
  {
    declare_parameter<std::string>("input_topic", "/cirtesub/sensors/gps/odometry");
    declare_parameter<std::string>("output_topic", "/cirtesub/sensors/gps/odometry_ned");
    declare_parameter<std::string>("frame_id", "world_ned");
    declare_parameter<std::string>("child_frame_id", "");

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic,
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        on_odometry(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Converting ENU odometry %s to NED %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  void on_odometry(const nav_msgs::msg::Odometry & msg)
  {
    nav_msgs::msg::Odometry converted;
    converted.header = msg.header;
    converted.header.frame_id = get_parameter("frame_id").as_string();
    converted.child_frame_id = get_parameter("child_frame_id").as_string();

    converted.pose.pose.position.x = msg.pose.pose.position.y;
    converted.pose.pose.position.y = msg.pose.pose.position.x;
    converted.pose.pose.position.z = -msg.pose.pose.position.z;
    converted.pose.pose.orientation = rotate_orientation(msg.pose.pose.orientation);
    converted.pose.covariance = transform_pose_covariance(msg.pose.covariance);

    converted.twist = msg.twist;
    converted.twist.twist.angular.x = msg.twist.twist.angular.y;
    converted.twist.twist.angular.y = msg.twist.twist.angular.x;
    converted.twist.twist.angular.z = -msg.twist.twist.angular.z;
    converted.twist.covariance = transform_twist_angular_covariance(msg.twist.covariance);

    publisher_->publish(converted);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnuToNedOdometry>());
  rclcpp::shutdown();
  return 0;
}
