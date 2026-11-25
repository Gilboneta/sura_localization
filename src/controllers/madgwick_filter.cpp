#include "sura_localization/controllers/madgwick_filter.hpp"

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
  const SensorSample &,
  const double)
{
  // The Madgwick equations will be implemented in the next step.
  return orientation_;
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
