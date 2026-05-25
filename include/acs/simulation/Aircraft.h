#pragma once

#include "acs/aero/AeroModel.h"
#include "acs/physics/RigidBody6Dof.h"

namespace acs::simulation {

struct AircraftConfig {
  acs::physics::RigidBodyParams rigid_body{};
  acs::aero::AeroParams aero{};
};

class Aircraft {
public:
  explicit Aircraft(const AircraftConfig& cfg);

  const acs::physics::RigidBody6Dof& rigid_body() const;
  const acs::aero::AeroModel& aero_model() const;

private:
  acs::physics::RigidBody6Dof rigid_body_;
  acs::aero::AeroModel aero_;
};

}  // namespace acs::simulation

