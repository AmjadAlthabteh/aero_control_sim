#include "acs/simulation/Wind.h"

#include <cmath>

namespace acs::simulation {

WindModel::WindModel(const WindConfig& cfg, unsigned int seed) : cfg_(cfg), noise_(seed) {}

void WindModel::reset(unsigned int seed) {
  noise_ = Noise(seed);
  gust_ned_m_s_ = acs::math::Vector3{};
}

const WindConfig& WindModel::cfg() const { return cfg_; }

acs::math::Vector3 WindModel::gust_ned_m_s() const { return gust_ned_m_s_; }

acs::math::Vector3 WindModel::wind_ned_m_s() const { return cfg_.mean_ned_m_s + gust_ned_m_s_; }

acs::math::Vector3 WindModel::update(double dt_s) {
  if (cfg_.gust_std_dev_m_s <= 0.0 || cfg_.gust_tau_s <= 1e-6 || dt_s <= 0.0) {
    gust_ned_m_s_ = acs::math::Vector3{};
    return wind_ned_m_s();
  }

  const double a = std::exp(-dt_s / cfg_.gust_tau_s);
  const double sigma = cfg_.gust_std_dev_m_s * std::sqrt(std::max(0.0, 1.0 - a * a));

  gust_ned_m_s_.x = a * gust_ned_m_s_.x + noise_.gaussian(0.0, sigma);
  gust_ned_m_s_.y = a * gust_ned_m_s_.y + noise_.gaussian(0.0, sigma);
  gust_ned_m_s_.z = a * gust_ned_m_s_.z + noise_.gaussian(0.0, sigma);

  return wind_ned_m_s();
}

}  // namespace acs::simulation

