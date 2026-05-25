#pragma once

namespace acs::physics {

struct AtmosphereState {
  double rho_kg_m3{};
  double pressure_pa{};
  double temperature_k{};
  double speed_of_sound_m_s{};
};

class Atmosphere {
public:
  // lightweight isa-like model near sea level.
  // altitude_m is positive up (asl). valid for typical low-altitude sims.
  static AtmosphereState at_altitude(double altitude_m);
};

}  // namespace acs::physics

