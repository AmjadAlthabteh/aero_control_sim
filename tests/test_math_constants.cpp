#include <cassert>
#include <cmath>

#include "acs/math/Constants.h"

namespace {
bool near(double a, double b, double eps = 1e-12) {
  return std::fabs(a - b) <= eps;
}
}  // namespace

int main() {
  using acs::math::kPi;
  using acs::math::kTwoPi;
  using acs::math::wrap_pi;

  assert(near(wrap_pi(0.0), 0.0));
  assert(near(wrap_pi(kPi), kPi));
  assert(near(wrap_pi(-kPi), kPi));
  assert(near(wrap_pi(3.0 * kPi), kPi));
  assert(near(wrap_pi(-3.0 * kPi), kPi));
  assert(near(wrap_pi(kTwoPi + 0.25), 0.25));
  assert(near(wrap_pi(-kTwoPi - 0.25), -0.25));
  assert(near(wrap_pi(std::numeric_limits<double>::quiet_NaN()), 0.0));

  return 0;
}
