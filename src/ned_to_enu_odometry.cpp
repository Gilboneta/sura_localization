#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <cmath>
#include <memory>
#include <string>

namespace
{

geometry_msgs::msg::Quaternion multiplyQuaternions(
  const geometry_msgs::msg::Quaternion & q1,
  const geometry_msgs::msg::Quaternion & q2)
{
  geometry_msgs::msg::Quaternion result;

  result.w =
    q1.w * q2.w -
    q1.x * q2.x -
    q1.y * q2.y -
    q1.z * q2.z;

  result.x =
    q1.w * q2.x +
    q1.x * q2.w +
    q1.y * q2.z -
    q1.z * q2.y;

  result.y =
    q1.w * q2.y -
    q1.x * q2.z +
    q1.y * q2.w +
    q1.z * q2.x;

  result.z =
    q1.w * q2.z +
    q1.x * q2.y -
    q1.y * q2.x +
    q1.z * q2.w;

  return result;
}

geometry_msgs::msg::Quaternion normalizeQuaternion(
  geometry_msgs::msg::Quaternion q)
{
  const double norm = std::sqrt(
    q.x * q.x +
    q.y * q.y +
    q.z * q.z +
    q.w * q.w);

  if (norm <= 0.0) {
    geometry_msgs::msg::Quaternion identity;
    identity.w = 1.0;
    return identity;
  }

  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;

  return q;
}

geometry_msgs::msg::Quaternion nedToEnuOrientation(
  const geometry_msgs::msg::Quaternion & orientation_ned)
{
  /*
   * Transformación NED -> ENU:
   *
   * x_enu =  y_ned
   * y_enu =  x_ned
   * z_enu = -z_ned
   *
   * Esta transformación equivale a una rotación de 180 grados
   * alrededor del eje (1,1,0)/sqrt(2).
   */

  geometry_msgs::msg::Quaternion q_ned_to_enu;

  q_ned_to_enu.x = std::sqrt(0.5);
  q_ned_to_enu.y = std::sqrt(0.5);
  q_ned_to_enu.z = 0.0;
  q_ned_to_enu.w = 0.0;

  return normalizeQuaternion(
    multiplyQuaternions(
      q_ned_to_enu,
      orientation_ned));
}

class NedToEnuOdometryNode : public rclcpp::Node
{
public:

  NedToEnuOdometryNode()
  : Node("ned_to_enu_odometry")
  {
    declare_parameter<std::string>(
      "input_topic",
      "/state_observer/predicted_odometry");

    declare_parameter<std::string>(
      "output_topic",
      "/state_observer/predicted_odometry_enu");

    declare_parameter<std::string>(
      "frame_id",
      "bluerov/map");

    declare_parameter<std::string>(
      "child_frame_id",
      "bluerov/base_link_flu");


    const std::string input_topic =
      get_parameter("input_topic").as_string();

    const std::string output_topic =
      get_parameter("output_topic").as_string();

    odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(
        output_topic,
        10);

    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
        input_topic,
        10,
        std::bind(
          &NedToEnuOdometryNode::odomCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Convirtiendo odometria NED %s -> ENU %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:

  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry odom_enu;

    odom_enu.header.stamp = msg->header.stamp;

    odom_enu.header.frame_id =
      get_parameter("frame_id").as_string();

    odom_enu.child_frame_id =
      get_parameter("child_frame_id").as_string();

    odom_enu.pose.pose.position.x =
      msg->pose.pose.position.y;

    odom_enu.pose.pose.position.y =
      msg->pose.pose.position.x;

    odom_enu.pose.pose.position.z =
      -msg->pose.pose.position.z;

    odom_enu.pose.pose.orientation =
      nedToEnuOrientation(
        msg->pose.pose.orientation);

    odom_enu.twist = msg->twist; //Dejamos la velocidad igual, ya que está expresada respecto a body

    odom_enu.pose.covariance =
      msg->pose.covariance;

    odom_enu.twist.covariance =
      msg->twist.covariance;

    odom_pub_->publish(odom_enu);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
    odom_pub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<NedToEnuOdometryNode>());

  rclcpp::shutdown();

  return 0;
}