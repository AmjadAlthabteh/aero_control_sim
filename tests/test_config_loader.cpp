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
    "controller": {
      "roll_gains": {"kp": 3.0, "ki": 0.2, "kd": 0.4, "kaw": 0.8, "tau_d": 0.05},
      "surface_limit_rad": 0.21,
      "throttle_limit": 0.77
    },
    "actuators": {
      "surface_limit_rad": 0.19
    }
  })");

  const auto params = acs::config::ConfigLoader::load_from_json(root);

  assert(near(params.sim_cfg.dt_s, 0.02));
  assert(near(params.sim_cfg.sim_time_s, 12.5));
  assert(near(params.ctrl_cfg.roll_gains.kp, 3.0));
  assert(near(params.ctrl_cfg.roll_gains.ki, 0.2));
  assert(near(params.ctrl_cfg.roll_gains.kd, 0.4));
  assert(near(params.ctrl_cfg.roll_gains.kaw, 0.8));
  assert(near(params.ctrl_cfg.roll_gains.tau_d, 0.05));
  assert(near(params.ctrl_cfg.surface_limit_rad, 0.21));
  assert(near(params.ctrl_cfg.throttle_limit, 0.77));
  assert(near(params.sim_cfg.actuators.surface_limit_rad, 0.19));

  return 0;
}
