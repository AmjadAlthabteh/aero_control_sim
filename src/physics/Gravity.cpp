#include "acs/physics/Gravity.h"

#include <cmath>

namespace acs::physics {

acs::math::Vector3 Gravity::ned(double altitude_m) {
  // spherical earth gravity model. "down" positive in ned.
  constexpr double g0 = 9.80665;
  constexpr double re_m = 6371000.0;
  const double g = g0 * std::pow(re_m / (re_m + altitude_m), 2.0);
  return acs::math::Vector3(0.0, 0.0, g);
}

}  // namespace acs::physics

