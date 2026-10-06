#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sura_msgs/msg/navigator.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

class SimpleStateObserverNode : public rclcpp::Node 
{
public:
  SimpleStateObserverNode() : Node("simple_state_observer") 
  {
    last_time_ = this->now();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/state_observer/predicted_odometry", 10);

    force_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>("/bluerov/controller/body_force/wrench", 10, 
    std::bind(&SimpleStateObserverNode::forceCallback, this, std::placeholders::_1));

    navigator_sub_ = this->create_subscription<sura_msgs::msg::Navigator>("/bluerov/navigator/navigation", 10, 
    std::bind(&SimpleStateObserverNode::NavigatorCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), 
      std::bind(&SimpleStateObserverNode::timerCallback, this)
    );
  }

private:

  using Vector6 = Eigen::Matrix<double, 6, 1>;

  bool initialized_ = false;

  Vector6 pose_ = Vector6::Zero();
  Vector6 body_velocity_ = Vector6::Zero();
  Vector6 wrench_ = Vector6::Zero();
  Vector6 restoring_wrench_ = Vector6::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();

  const double weight_ = 112.8;
  const double buoyancy_ = 121.3;
  Vector6 mass_ = (Vector6() << 150.0, 150.0, 210.0, 0.28, 0.28, 0.28).finished();
  Vector6 linear_damping_ = (Vector6() << 105.0, 95.0, 130.0, 0.07, 0.07, 0.07).finished();
  Vector6 quadratic_damping_ = (Vector6() << 85.0, 140.0, 210.0, 1.55, 1.55, 1.55).finished();

  rclcpp::Time last_time_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr force_sub_;
  rclcpp::Subscription<sura_msgs::msg::Navigator>::SharedPtr navigator_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void forceCallback(const geometry_msgs::msg::Wrench::SharedPtr msg)
  {
    wrench_(0) = msg->force.x;
    wrench_(1) = msg->force.y;
    wrench_(2) = msg->force.z; 
    wrench_(3) = msg->torque.x;
    wrench_(4) = msg->torque.y;
    wrench_(5) = msg->torque.z;

  }

  void NavigatorCallback(const sura_msgs::msg::Navigator::SharedPtr msg)
  {
    orientation_.x() = msg->position.orientation.x;
    orientation_.y() = msg->position.orientation.y;
    orientation_.z() = msg->position.orientation.z;
    orientation_.w() = msg->position.orientation.w;

    orientation_.normalize(); //Normalizamos el quaternion

    if (!initialized_) { //Cuando se recibe el primer dato de la Navigator, se inicializa la pose y velocidad inciales
      pose_(0) = msg->position.position.x;
      pose_(1) = msg->position.position.y;
      pose_(2) = msg->position.position.z;

      body_velocity_(0) = msg->body_velocity.linear.x;
      body_velocity_(1) = msg->body_velocity.linear.y;
      body_velocity_(2) = msg->body_velocity.linear.z;

      body_velocity_(3) = msg->body_velocity.angular.x;
      body_velocity_(4) = msg->body_velocity.angular.y;
      body_velocity_(5) = msg->body_velocity.angular.z;

      last_time_ = this->now();
      initialized_ = true; //Cambiamos initialized_ a True para que no vuelva a entrar al if
    }
  }

  void timerCallback()
  {

    if (!initialized_) {
        return;
    }

    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 0.1) {
        return;
    }

    double restoring_z = buoyancy_ - weight_;

    if (pose_(2) <= 0.0) {
      restoring_z = 0.0;
    }

    Eigen::Vector3d restoring_ned(0.0, 0.0, restoring_z);

    Eigen::Vector3d restoring_body = orientation_.inverse() * restoring_ned;

    restoring_wrench_.setZero();
    restoring_wrench_.head<3>() = restoring_body;

    for (int i = 0; i < 6; i++) {
        double damping_force = linear_damping_(i) * body_velocity_(i) + 
                               quadratic_damping_(i) * std::abs(body_velocity_(i)) * body_velocity_(i);
        double acceleration = (wrench_(i) - damping_force - restoring_wrench_(i)) / mass_(i);
        body_velocity_(i) += acceleration * dt;
    }

    Eigen::Vector3d velocity_ned =
    orientation_ * body_velocity_.head<3>(); //Pasamos la velocidad al world frame

    pose_.head<3>() += velocity_ned * dt; //Obtenemos la posicion respecto al world frame

    // No permitimos que el modelo atraviese la superficie
    if (pose_(2) < 0.0) {
      pose_(2) = 0.0;

      // Si está intentando moverse hacia arriba, anulamos
      // únicamente la velocidad vertical en NED.
      if (velocity_ned.z() < 0.0) {
        velocity_ned.z() = 0.0;

        // Volvemos a BODY
        body_velocity_.head<3>() =orientation_.inverse() * velocity_ned;
      }
    }

    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "bluerov/map";
    odom_msg.child_frame_id = "bluerov/base_link";

    odom_msg.pose.pose.position.x = pose_(0);
    odom_msg.pose.pose.position.y = pose_(1);
    odom_msg.pose.pose.position.z = pose_(2);

    odom_msg.pose.pose.orientation.x = orientation_.x();
    odom_msg.pose.pose.orientation.y = orientation_.y();
    odom_msg.pose.pose.orientation.z = orientation_.z();
    odom_msg.pose.pose.orientation.w = orientation_.w();

    odom_msg.twist.twist.linear.x = body_velocity_(0);
    odom_msg.twist.twist.linear.y = body_velocity_(1);
    odom_msg.twist.twist.linear.z = body_velocity_(2);

    odom_msg.twist.twist.angular.x = body_velocity_(3);
    odom_msg.twist.twist.angular.y = body_velocity_(4);
    odom_msg.twist.twist.angular.z = body_velocity_(5);

    odom_msg.pose.covariance.fill(0.0);
    odom_msg.pose.covariance[0] = 0.1;
    odom_msg.pose.covariance[7] = 0.1;
    odom_msg.pose.covariance[14] = 0.1;
    odom_msg.pose.covariance[21] = 0.05;
    odom_msg.pose.covariance[28] = 0.05;
    odom_msg.pose.covariance[35] = 0.05;

    odom_msg.twist.covariance.fill(0.0);
    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[7] = 0.01;
    odom_msg.twist.covariance[14] = 0.01;
    odom_msg.twist.covariance[21] = 0.01;
    odom_msg.twist.covariance[28] = 0.01;
    odom_msg.twist.covariance[35] = 0.01;

    odom_pub_->publish(odom_msg);
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleStateObserverNode>());
  rclcpp::shutdown();
  return 0;
}