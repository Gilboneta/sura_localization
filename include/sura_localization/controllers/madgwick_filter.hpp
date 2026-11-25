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

  struct SensorSample
  {
    Vector3 gyroscope{Vector3::Zero()};
    Vector3 acceleration{Vector3::Zero()};
    Vector3 magnetic_field{Vector3::Zero()};
  };

  explicit MadgwickFilter(double beta = 0.1);

  void reset(const Quaternion & initial_orientation = Quaternion::Identity());

  Quaternion update(const SensorSample & sample, double dt);

  const Quaternion & orientation() const;

  void set_beta(double beta);

  double beta() const;

private:
  Quaternion orientation_{Quaternion::Identity()};
  double beta_{0.1};
};

}  // namespace sura_localization::controllers
