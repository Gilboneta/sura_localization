#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

struct Quat
{
  double x;
  double y;
  double z;
  double w;
};

static Quat normalizeQuat(const Quat & q)
{
  const double n = std::sqrt(
    q.x * q.x +
    q.y * q.y +
    q.z * q.z +
    q.w * q.w
  );

  if (n < 1e-12) {
    return {0.0, 0.0, 0.0, 1.0};
  }

  return {
    q.x / n,
    q.y / n,
    q.z / n,
    q.w / n
  };
}

static Quat multiplyQuat(const Quat & a, const Quat & b)
{
  Quat q;

  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

  return normalizeQuat(q);
}

class ImuNedToEnuNode : public rclcpp::Node
{
public:
  ImuNedToEnuNode()
  : Node("imu_ned_to_enu")
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic",
      "/bluerov/sensors/imu"
    );

    output_topic_ = this->declare_parameter<std::string>(
      "output_topic",
      "/bluerov/sensors/imu_enu"
    );

    output_frame_ = this->declare_parameter<std::string>(
      "output_frame",
      "bluerov/imu_link_flu"
    );

    convert_frd_to_flu_ = this->declare_parameter<bool>(
      "convert_frd_to_flu",
      true
    );

    orientation_yaw_stddev_deg_ = this->declare_parameter<double>(
      "orientation_yaw_stddev_deg",
      -1.0
    );

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      output_topic_,
      rclcpp::SensorDataQoS()
    );

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ImuNedToEnuNode::imuCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "IMU NED/FRD -> ENU/FLU converter started");
    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_frame: %s", output_frame_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "convert_frd_to_flu: %s",
      convert_frd_to_flu_ ? "true" : "false"
    );
  }

private:
  static geometry_msgs::msg::Vector3 frdToFluVector(
    const geometry_msgs::msg::Vector3 & v)
  {
    geometry_msgs::msg::Vector3 out;

    out.x = v.x;
    out.y = -v.y;
    out.z = -v.z;

    return out;
  }

  static void frdToFluCovariance(std::array<double, 9> & cov)
  {
    const double s[3] = {1.0, -1.0, -1.0};

    std::array<double, 9> in = cov;

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        cov[3 * r + c] = s[r] * in[3 * r + c] * s[c];
      }
    }
  }

  void maybeOverrideOrientationYawCovariance(sensor_msgs::msg::Imu & out)
  {
    if (orientation_yaw_stddev_deg_ < 0.0) {
      return;
    }

    constexpr double pi = 3.14159265358979323846;
    const double yaw_stddev_rad =
      orientation_yaw_stddev_deg_ * pi / 180.0;

    const double yaw_var = yaw_stddev_rad * yaw_stddev_rad;

    /*
     * Si la IMU raw viene con orientation_covariance[0] = -1,
     * robot_localization puede ignorar la orientación completa.
     *
     * Si tú has dado imu_orientation_yaw_stddev_deg >= 0,
     * hacemos que la orientación sea utilizable.
     *
     * Roll/pitch reciben varianza muy grande para que no pesen,
     * salvo que tu configuración de robot_localization los fusione a propósito.
     */
    if (out.orientation_covariance[0] == -1.0) {
      out.orientation_covariance = {
        1e6, 0.0, 0.0,
        0.0, 1e6, 0.0,
        0.0, 0.0, yaw_var
      };
    } else {
      out.orientation_covariance[8] = yaw_var;
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out = *msg;

    /*
     * El mensaje convertido ya no está en imu_link raw.
     * Está en el frame corregido FLU.
     */
    out.header.frame_id = output_frame_;

    /*
     * q_raw:
     *   orientación del sensor FRD respecto al mundo NED
     *
     * q_out:
     *   orientación del sensor FLU respecto al mundo ENU
     *
     * Fórmula:
     *
     *   q_out = q_enu_ned * q_raw * q_frd_flu
     */
    const auto & q_msg = msg->orientation;

    const double norm_sq =
      q_msg.x * q_msg.x +
      q_msg.y * q_msg.y +
      q_msg.z * q_msg.z +
      q_msg.w * q_msg.w;

    if (norm_sq > 1e-12) {
      Quat q_raw{
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
      };

      q_raw = normalizeQuat(q_raw);

      constexpr double k = 0.7071067811865476;

      /*
       * Mundo NED -> mundo ENU.
       *
       * Coordenadas:
       *   x_enu =  y_ned
       *   y_enu =  x_ned
       *   z_enu = -z_ned
       *
       * Quaternion ROS: x, y, z, w
       */
      const Quat q_enu_ned{k, k, 0.0, 0.0};

      /*
       * Sensor/body FRD -> FLU.
       *
       * FRD:
       *   x forward
       *   y right
       *   z down
       *
       * FLU:
       *   x forward
       *   y left
       *   z up
       *
       * Equivale a roll = pi.
       */
      const Quat q_frd_flu = convert_frd_to_flu_
        ? Quat{1.0, 0.0, 0.0, 0.0}
        : Quat{0.0, 0.0, 0.0, 1.0};

      Quat q_out = multiplyQuat(q_enu_ned, q_raw);
      q_out = multiplyQuat(q_out, q_frd_flu);
      q_out = normalizeQuat(q_out);

      out.orientation.x = q_out.x;
      out.orientation.y = q_out.y;
      out.orientation.z = q_out.z;
      out.orientation.w = q_out.w;
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Received IMU quaternion with near-zero norm. Orientation not converted."
      );
    }

    /*
     * angular_velocity y linear_acceleration están expresadas
     * en el frame del sensor.
     *
     * Si cambiamos de FRD a FLU:
     *
     *   x_out =  x_in
     *   y_out = -y_in
     *   z_out = -z_in
     */
    if (convert_frd_to_flu_) {
      out.angular_velocity = frdToFluVector(msg->angular_velocity);
      out.linear_acceleration = frdToFluVector(msg->linear_acceleration);

      frdToFluCovariance(out.angular_velocity_covariance);
      frdToFluCovariance(out.linear_acceleration_covariance);

      if (out.orientation_covariance[0] != -1.0) {
        frdToFluCovariance(out.orientation_covariance);
      }
    }

    maybeOverrideOrientationYawCovariance(out);

    imu_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;

  bool convert_frd_to_flu_;
  double orientation_yaw_stddev_deg_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNedToEnuNode>());
  rclcpp::shutdown();

  return 0;
}