#include "acs/simulation/Aircraft.h"

namespace acs::simulation {

Aircraft::Aircraft(const AircraftConfig& cfg) : rigid_body_(cfg.rigid_body), aero_(cfg.aero) {}

const acs::physics::RigidBody6Dof& Aircraft::rigid_body() const { return rigid_body_; }

const acs::aero::AeroModel& Aircraft::aero_model() const { return aero_; }

}  // namespace acs::simulation

