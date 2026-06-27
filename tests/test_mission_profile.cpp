#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "acs/math/Constants.h"
#include "acs/simulation/MissionProfile.h"

namespace {
bool near(double a, double b, double eps = 1e-12) {
  return std::fabs(a - b) <= eps;
}
}  // namespace

int main() {
  using acs::math::kDeg2Rad;
  using acs::simulation::MissionProfile;
  using acs::simulation::MissionWaypoint;

  acs::gnc::ControllerTargets a{};
  a.altitude_m = 60.0;
  a.heading_rad = 170.0 * kDeg2Rad;
  a.airspeed_m_s = 16.0;
  a.roll_rad = 0.0;
  a.pitch_rad = 1.0 * kDeg2Rad;

  acs::gnc::ControllerTargets b{};
  b.altitude_m = 120.0;
  b.heading_rad = -170.0 * kDeg2Rad;
  b.airspeed_m_s = 22.0;
  b.roll_rad = 10.0 * kDeg2Rad;
  b.pitch_rad = 3.0 * kDeg2Rad;

  MissionProfile mission({MissionWaypoint{10.0, b}, MissionWaypoint{0.0, a}});

  assert(mission.size() == 2);
  assert(near(mission.target_at(-1.0).altitude_m, 60.0));
  assert(near(mission.target_at(20.0).altitude_m, 120.0));

  const auto mid = mission.target_at(5.0);
  assert(near(mid.altitude_m, 90.0));
  assert(near(mid.airspeed_m_s, 19.0));
  assert(near(mid.roll_rad, 5.0 * kDeg2Rad));
  assert(near(mid.pitch_rad, 2.0 * kDeg2Rad));
  assert(near(std::fabs(mid.heading_rad), acs::math::kPi, 1e-12));
  assert(mission.segment_index_at(5.0) == 0);
  assert(mission.segment_index_at(10.0) == 1);

  bool rejected_duplicate_time = false;
  try {
    MissionProfile invalid({MissionWaypoint{0.0, a}, MissionWaypoint{0.0, b}});
  } catch (const std::runtime_error&) {
    rejected_duplicate_time = true;
  }
  assert(rejected_duplicate_time);

  return 0;
}
