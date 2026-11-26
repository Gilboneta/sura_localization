#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sura_localization::controllers
{

class MadgwickFilter
{
public:
  using Vector3 = Eigen::Vector3d;
  using Quaternion = Eigen::Quaterniond;

  using Vector4 = Eigen::Vector4d;

  // Sensor sample expressed in the body frame (FRD).
  // The orientation estimated by the filter is body (FRD) -> world (NED).
  struct SensorSample
  {
    Vector3 gyroscope{Vector3::Zero()};       // [rad/s]
    Vector3 acceleration{Vector3::Zero()};    // specific force, [0, 0, -g] at rest and level
    Vector3 magnetic_field{Vector3::Zero()};  // any unit, only its direction is used
    bool use_magnetic_field{true};            // false -> IMU-only correction for this sample
  };

  explicit MadgwickFilter(double beta = 0.1);

  void reset(const Quaternion & initial_orientation = Quaternion::Identity());

  Quaternion update(const SensorSample & sample, double dt);

  const Quaternion & orientation() const;

  void set_beta(double beta);

  double beta() const;

private:
  // Normalized gradient of the Madgwick objective function, in (w, x, y, z) order.
  // Returns false when no correction can be computed (invalid or degenerate measurements).
  bool compute_correction_step(const SensorSample & sample, Vector4 & step) const;

  Quaternion orientation_{Quaternion::Identity()};
  double beta_{0.1};
};

}  // namespace sura_localization::controllers
