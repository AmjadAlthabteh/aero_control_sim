#pragma once

#include "acs/gnc/FlightController.h"

#include <cstddef>
#include <vector>

namespace acs::simulation {

struct MissionWaypoint {
  double time_s{0.0};
  acs::gnc::ControllerTargets targets{};
};

class MissionProfile {
public:
  MissionProfile() = default;
  explicit MissionProfile(const acs::gnc::ControllerTargets& targets);
  explicit MissionProfile(std::vector<MissionWaypoint> waypoints);

  static MissionProfile hold(const acs::gnc::ControllerTargets& targets);

  bool empty() const;
  std::size_t size() const;
  const std::vector<MissionWaypoint>& waypoints() const;

  acs::gnc::ControllerTargets target_at(double time_s) const;
  std::size_t segment_index_at(double time_s) const;

private:
  std::vector<MissionWaypoint> waypoints_{};

  void normalize();
};

}  // namespace acs::simulation
