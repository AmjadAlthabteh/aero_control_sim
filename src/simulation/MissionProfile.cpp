#include "acs/simulation/MissionProfile.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "acs/math/Constants.h"

namespace acs::simulation {
namespace {

double normalize_angle_rad(double angle_rad) {
  constexpr double two_pi = 2.0 * acs::math::kPi;
  while (angle_rad > acs::math::kPi) {
    angle_rad -= two_pi;
  }
  while (angle_rad < -acs::math::kPi) {
    angle_rad += two_pi;
  }
  return angle_rad;
}

double lerp(double a, double b, double u) {
  return a + (b - a) * u;
}

acs::gnc::ControllerTargets interpolate_targets(const acs::gnc::ControllerTargets& a,
                                                const acs::gnc::ControllerTargets& b,
                                                double u) {
  acs::gnc::ControllerTargets out{};
  out.roll_rad = lerp(a.roll_rad, b.roll_rad, u);
  out.pitch_rad = lerp(a.pitch_rad, b.pitch_rad, u);
  out.altitude_m = lerp(a.altitude_m, b.altitude_m, u);
  out.airspeed_m_s = lerp(a.airspeed_m_s, b.airspeed_m_s, u);

  const double heading_delta = normalize_angle_rad(b.heading_rad - a.heading_rad);
  out.heading_rad = normalize_angle_rad(a.heading_rad + heading_delta * u);
  return out;
}

}  // namespace

MissionProfile::MissionProfile(const acs::gnc::ControllerTargets& targets)
    : waypoints_{{0.0, targets}} {}

MissionProfile::MissionProfile(std::vector<MissionWaypoint> waypoints)
    : waypoints_(std::move(waypoints)) {
  normalize();
}

MissionProfile MissionProfile::hold(const acs::gnc::ControllerTargets& targets) {
  return MissionProfile(targets);
}

bool MissionProfile::empty() const {
  return waypoints_.empty();
}

std::size_t MissionProfile::size() const {
  return waypoints_.size();
}

const std::vector<MissionWaypoint>& MissionProfile::waypoints() const {
  return waypoints_;
}

acs::gnc::ControllerTargets MissionProfile::target_at(double time_s) const {
  if (waypoints_.empty()) {
    return acs::gnc::ControllerTargets{};
  }
  if (time_s <= waypoints_.front().time_s || waypoints_.size() == 1) {
    return waypoints_.front().targets;
  }
  if (time_s >= waypoints_.back().time_s) {
    return waypoints_.back().targets;
  }

  const auto upper = std::upper_bound(
      waypoints_.begin(), waypoints_.end(), time_s,
      [](double t, const MissionWaypoint& waypoint) { return t < waypoint.time_s; });
  const auto lower = upper - 1;
  const double dt = upper->time_s - lower->time_s;
  const double u = dt > 0.0 ? (time_s - lower->time_s) / dt : 0.0;
  return interpolate_targets(lower->targets, upper->targets, u);
}

std::size_t MissionProfile::segment_index_at(double time_s) const {
  if (waypoints_.empty() || waypoints_.size() == 1 || time_s <= waypoints_.front().time_s) {
    return 0;
  }
  if (time_s >= waypoints_.back().time_s) {
    return waypoints_.size() - 1;
  }
  const auto upper = std::upper_bound(
      waypoints_.begin(), waypoints_.end(), time_s,
      [](double t, const MissionWaypoint& waypoint) { return t < waypoint.time_s; });
  return static_cast<std::size_t>((upper - waypoints_.begin()) - 1);
}

void MissionProfile::normalize() {
  if (waypoints_.empty()) {
    throw std::runtime_error("Mission profile must contain at least one waypoint");
  }

  std::sort(waypoints_.begin(), waypoints_.end(), [](const MissionWaypoint& a, const MissionWaypoint& b) {
    return a.time_s < b.time_s;
  });

  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    const auto& waypoint = waypoints_[i];
    if (!std::isfinite(waypoint.time_s) || waypoint.time_s < 0.0) {
      throw std::runtime_error("Mission waypoint time must be finite and non-negative");
    }
    if (i > 0 && waypoint.time_s <= waypoints_[i - 1].time_s) {
      throw std::runtime_error("Mission waypoint times must be unique");
    }
    if (!std::isfinite(waypoint.targets.altitude_m) ||
        !std::isfinite(waypoint.targets.heading_rad) ||
        !std::isfinite(waypoint.targets.airspeed_m_s) ||
        !std::isfinite(waypoint.targets.roll_rad) ||
        !std::isfinite(waypoint.targets.pitch_rad)) {
      throw std::runtime_error("Mission waypoint targets must be finite");
    }
    if (waypoint.targets.airspeed_m_s <= 0.0) {
      throw std::runtime_error("Mission waypoint airspeed must be positive");
    }
  }
}

}  // namespace acs::simulation
