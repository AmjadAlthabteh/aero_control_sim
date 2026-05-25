#pragma once

namespace acs::math {

// mathematical constants
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kHalfPi = 0.5 * kPi;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

// physical constants
constexpr double kGravityMsl_m_s2 = 9.80665;       // standard gravity at MSL
constexpr double kEarthRadius_m = 6371000.0;       // mean Earth radius

}  // namespace acs::math
