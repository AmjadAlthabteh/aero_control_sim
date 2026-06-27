#include <cassert>
#include <cmath>

#include "acs/config/ConfigLoader.h"

namespace {
bool near(double a, double b, double eps = 1e-12) {
  return std::fabs(a - b) <= eps;
}
}  // namespace

int main() {
  const auto root = acs::config::JsonParser::parse(R"({
    "dt_s": 0.02,
    "sim_time_s": 12.5,
    "max_steps": 250,
    "controller": {
      "roll_gains": {"kp": 3.0, "ki": 0.2, "kd": 0.4, "kaw": 0.8, "tau_d": 0.05},
      "surface_limit_rad": 0.21,
      "throttle_limit": 0.77
    },
    "targets": {
      "altitude_m": 60.0,
      "heading_deg": 10.0,
      "airspeed_m_s": 17.0
    },
    "mission": {
      "waypoints": [
        {"time_s": 0.0},
        {"time_s": 5.0, "altitude_m": 90.0, "heading_deg": 30.0},
        {"time_s": 10.0, "airspeed_m_s": 21.0}
      ]
    },
    "actuators": {
      "surface_limit_rad": 0.19,
      "surface_deadband_rad": 0.02
    }
  })");

  const auto params = acs::config::ConfigLoader::load_from_json(root);

  assert(near(params.sim_cfg.dt_s, 0.02));
  assert(near(params.sim_cfg.sim_time_s, 12.5));
  assert(params.sim_cfg.max_steps == 250);
  assert(near(params.ctrl_cfg.roll_gains.kp, 3.0));
  assert(near(params.ctrl_cfg.roll_gains.ki, 0.2));
  assert(near(params.ctrl_cfg.roll_gains.kd, 0.4));
  assert(near(params.ctrl_cfg.roll_gains.kaw, 0.8));
  assert(near(params.ctrl_cfg.roll_gains.tau_d, 0.05));
  assert(near(params.ctrl_cfg.surface_limit_rad, 0.21));
  assert(near(params.ctrl_cfg.throttle_limit, 0.77));
  assert(near(params.sim_cfg.actuators.surface_limit_rad, 0.19));
  assert(near(params.sim_cfg.actuators.surface_deadband_rad, 0.02));
  assert(params.mission.size() == 3);
  assert(near(params.mission.target_at(0.0).altitude_m, 60.0));
  assert(near(params.mission.target_at(2.5).altitude_m, 75.0));
  assert(near(params.mission.target_at(7.5).altitude_m, 90.0));
  assert(near(params.mission.target_at(7.5).airspeed_m_s, 19.0));
  assert(near(params.mission.target_at(5.0).heading_rad, 30.0 * acs::math::kDeg2Rad));

  return 0;
}
