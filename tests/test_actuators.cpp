#include <cassert>
#include <cmath>

#include "acs/simulation/Actuators.h"

namespace {
bool near(double a, double b, double eps = 1e-12) {
  return std::fabs(a - b) <= eps;
}
}  // namespace

int main() {
  acs::simulation::ActuatorConfig cfg{};
  cfg.surface_deadband_rad = 0.01;
  cfg.surface_tau_s = 0.0;
  cfg.surface_rate_limit_rad_s = 1e9;

  acs::simulation::Actuators actuators(cfg);
  acs::aero::ControlInputs cmd{};

  cmd.aileron_rad = 0.005;
  auto out = actuators.update(cmd, 0.01, 0.0);
  assert(near(out.aileron_rad, 0.0));

  cmd.aileron_rad = 0.03;
  out = actuators.update(cmd, 0.01, 0.01);
  assert(near(out.aileron_rad, 0.02));

  return 0;
}
