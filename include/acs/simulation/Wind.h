#pragma once

#include "acs/math/Vector3.h"
#include "acs/simulation/Noise.h"

namespace acs::simulation {

struct WindConfig {
  // steady wind in navigation frame (ned), m/s.
  acs::math::Vector3 mean_ned_m_s{};

  // 1-sigma gust magnitude per axis (stationary), m/s.
  double gust_std_dev_m_s{0.0};

  // gust correlation time constant, seconds.
  double gust_tau_s{2.0};
};

class WindModel {
public:
  explicit WindModel(const WindConfig& cfg = WindConfig{}, unsigned int seed = 1U);

  void reset(unsigned int seed = 1U);
  acs::math::Vector3 update(double dt_s);

  const WindConfig& cfg() const;
  acs::math::Vector3 wind_ned_m_s() const;
  acs::math::Vector3 gust_ned_m_s() const;

private:
  WindConfig cfg_{};
  Noise noise_{};
  acs::math::Vector3 gust_ned_m_s_{};
};

}  // namespace acs::simulation

