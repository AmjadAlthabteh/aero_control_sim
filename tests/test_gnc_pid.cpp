#include <cassert>
#include <cmath>
#include <limits>

#include "acs/gnc/PID.h"

int main() {
  acs::gnc::PID::Gains gains{};
  gains.kp = 2.0;
  gains.ki = 0.5;
  gains.kd = 0.1;
  acs::gnc::PID pid(gains, 1.0, 1.0);

  assert(pid.update(1.0, std::numeric_limits<double>::quiet_NaN(), 0.01) == 0.0);
  assert(pid.update(std::numeric_limits<double>::infinity(), 0.0, 0.01) == 0.0);

  // Invalid samples do not poison persistent state; normal control resumes on
  // the next finite sample.
  const double recovered = pid.update(1.0, 0.0, 0.01);
  assert(std::isfinite(recovered));
  assert(recovered > 0.0);
  return 0;
}
