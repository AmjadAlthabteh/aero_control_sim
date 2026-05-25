#include "acs/physics/Atmosphere.h"

#include <algorithm>
#include <cmath>

namespace acs::physics {

AtmosphereState Atmosphere::at_altitude(double altitude_m) {
  // international standard atmosphere (troposphere) with a mild clamp.
  // this is plenty for low-altitude aircraft/drone scenarios.
  constexpr double t0 = 288.15;         // k
  constexpr double p0 = 101325.0;       // pa
  constexpr double rho0 = 1.225;        // kg/m^3
  constexpr double lapse = 0.0065;      // k/m
  constexpr double g0 = 9.80665;        // m/s^2
  constexpr double r_air = 287.05287;   // j/(kg*k)
  constexpr double gamma = 1.4;

  const double h = std::clamp(altitude_m, -500.0, 20000.0);

  AtmosphereState s{};
  if (h <= 11000.0) {
    s.temperature_k = t0 - lapse * h;
    s.pressure_pa = p0 * std::pow(s.temperature_k / t0, g0 / (r_air * lapse));
    s.rho_kg_m3 = s.pressure_pa / (r_air * s.temperature_k);
  } else {
    // simple exponential extension above the tropopause.
    s.temperature_k = t0 - lapse * 11000.0;
    const double p11 = p0 * std::pow(s.temperature_k / t0, g0 / (r_air * lapse));
    const double rho11 = p11 / (r_air * s.temperature_k);
    const double hscale = 7000.0;
    const double dh = h - 11000.0;
    s.rho_kg_m3 = rho11 * std::exp(-dh / hscale);
    s.pressure_pa = s.rho_kg_m3 * r_air * s.temperature_k;
  }

  s.speed_of_sound_m_s = std::sqrt(gamma * r_air * s.temperature_k);
  (void)rho0;
  return s;
}

}  // namespace acs::physics

