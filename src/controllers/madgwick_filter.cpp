#include "sura_localization/controllers/madgwick_filter.hpp"

#include <cmath>
#include <stdexcept>

namespace sura_localization::controllers
{

MadgwickFilter::MadgwickFilter(const double beta)
: beta_(beta)
{
  if (beta < 0.0) {
    throw std::invalid_argument("Madgwick beta must be non-negative");
  }
}

void MadgwickFilter::reset(const Quaternion & initial_orientation)
{
  orientation_ = initial_orientation.normalized();
}

MadgwickFilter::Quaternion MadgwickFilter::update(
  const SensorSample & sample,
  const double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0 || !sample.gyroscope.allFinite()) {
    return orientation_;
  }

  // 1) Prediction with the gyroscope.
  // Angular velocity is represented as a pure quaternion: (0, wx, wy, wz).
  const Quaternion angular_velocity_quaternion(
    0.0,
    sample.gyroscope.x(),
    sample.gyroscope.y(),
    sample.gyroscope.z());

  // Quaternion kinematics for a body-to-world orientation:
  // q_dot = 1/2 * q * omega.
  const Quaternion gyro_derivative = orientation_ * angular_velocity_quaternion;

  // Work in (w, x, y, z) order. Eigen stores coeffs() as (x, y, z, w).
  Vector4 quaternion_derivative(
    0.5 * gyro_derivative.w(),
    0.5 * gyro_derivative.x(),
    0.5 * gyro_derivative.y(),
    0.5 * gyro_derivative.z());

  // 2) Correction with accelerometer (+ magnetometer): one gradient descent
  // step towards the orientation that best explains the measured directions.
  // q_dot = q_dot_gyro - beta * grad(f) / |grad(f)|
  Vector4 correction_step;
  if (beta_ > 0.0 && compute_correction_step(sample, correction_step)) {
    quaternion_derivative -= beta_ * correction_step;
  }

  // 3) First-order Euler integration.
  orientation_.w() += quaternion_derivative(0) * dt;
  orientation_.x() += quaternion_derivative(1) * dt;
  orientation_.y() += quaternion_derivative(2) * dt;
  orientation_.z() += quaternion_derivative(3) * dt;

  // Numerical integration can move the quaternion away from unit length.
  // A unit quaternion is required to represent a valid orientation.
  const double orientation_norm = orientation_.norm();
  if (!std::isfinite(orientation_norm) || orientation_norm <= 1.0e-12) {
    orientation_.setIdentity();
  } else {
    orientation_.normalize();
  }

  return orientation_;
}

bool MadgwickFilter::compute_correction_step(
  const SensorSample & sample,
  Vector4 & step) const
{
  constexpr double kMinNorm = 1.0e-9;

  if (!sample.acceleration.allFinite()) {
    return false;
  }

  const double acceleration_norm = sample.acceleration.norm();
  if (acceleration_norm < kMinNorm) {
    return false;
  }

  // World frame is NED. At rest the accelerometer measures the specific force
  // [0, 0, -g] (pointing up). The original Madgwick equations use a z-up
  // gravity reference [0, 0, 1]; using -a with that reference produces exactly
  // the same gradient as using a with the NED reference [0, 0, -1]
  // (f and J both change sign, so J^T * f is unchanged).
  const Vector3 a = -sample.acceleration / acceleration_norm;

  const double w = orientation_.w();
  const double x = orientation_.x();
  const double y = orientation_.y();
  const double z = orientation_.z();

  // Gravity objective: f_g = R(q)^T * [0, 0, 1] - a
  const Vector3 f_g(
    2.0 * (x * z - w * y) - a.x(),
    2.0 * (w * x + y * z) - a.y(),
    2.0 * (0.5 - x * x - y * y) - a.z());

  // Jacobian of f_g with respect to (w, x, y, z).
  Eigen::Matrix<double, 3, 4> J_g;
  J_g <<
    -2.0 * y, 2.0 * z, -2.0 * w, 2.0 * x,
    2.0 * x, 2.0 * w, 2.0 * z, 2.0 * y,
    0.0, -4.0 * x, -4.0 * y, 0.0;

  Vector4 gradient = J_g.transpose() * f_g;

  // Magnetometer objective (MARG). Only used when the sample is valid.
  const double magnetic_field_norm = sample.magnetic_field.norm();
  if (sample.use_magnetic_field &&
    sample.magnetic_field.allFinite() &&
    magnetic_field_norm > kMinNorm)
  {
    const Vector3 m = sample.magnetic_field / magnetic_field_norm;

    // Reference direction of the Earth's field in the world frame: rotate the
    // measurement to world and keep only its horizontal magnitude (north, x)
    // and vertical component (z). This removes any dependency on the field
    // direction (declination), so the magnetometer only corrects yaw.
    const Vector3 h = orientation_ * m;
    const double bx = std::sqrt(h.x() * h.x() + h.y() * h.y());
    const double bz = h.z();

    // Magnetic objective: f_b = R(q)^T * [bx, 0, bz] - m
    const Vector3 f_b(
      2.0 * bx * (0.5 - y * y - z * z) + 2.0 * bz * (x * z - w * y) - m.x(),
      2.0 * bx * (x * y - w * z) + 2.0 * bz * (w * x + y * z) - m.y(),
      2.0 * bx * (w * y + x * z) + 2.0 * bz * (0.5 - x * x - y * y) - m.z());

    // Jacobian of f_b with respect to (w, x, y, z).
    Eigen::Matrix<double, 3, 4> J_b;
    J_b <<
      -2.0 * bz * y, 2.0 * bz * z, -4.0 * bx * y - 2.0 * bz * w, -4.0 * bx * z + 2.0 * bz * x,
      -2.0 * bx * z + 2.0 * bz * x, 2.0 * bx * y + 2.0 * bz * w, 2.0 * bx * x + 2.0 * bz * z,
      -2.0 * bx * w + 2.0 * bz * y,
      2.0 * bx * y, 2.0 * bx * z - 4.0 * bz * x, 2.0 * bx * w - 4.0 * bz * y, 2.0 * bx * x;

    gradient += J_b.transpose() * f_b;
  }

  const double gradient_norm = gradient.norm();
  if (!std::isfinite(gradient_norm) || gradient_norm < kMinNorm) {
    return false;
  }

  step = gradient / gradient_norm;
  return true;
}

const MadgwickFilter::Quaternion & MadgwickFilter::orientation() const
{
  return orientation_;
}

void MadgwickFilter::set_beta(const double beta)
{
  if (beta < 0.0) {
    throw std::invalid_argument("Madgwick beta must be non-negative");
  }

  beta_ = beta;
}

double MadgwickFilter::beta() const
{
  return beta_;
}

}  // namespace sura_localization::controllers
