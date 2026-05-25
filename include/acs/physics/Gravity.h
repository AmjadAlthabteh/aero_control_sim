#pragma once

#include "acs/math/Vector3.h"

namespace acs::physics {

class Gravity {
public:
  // returns gravity acceleration vector in ned (down positive) in m/s^2.
  static acs::math::Vector3 ned(double altitude_m);
};

}  // namespace acs::physics

