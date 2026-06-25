#pragma once

#include "acs/aero/AeroModel.h"
#include "acs/gnc/FlightController.h"
#include "acs/physics/RigidBody6Dof.h"
#include "acs/simulation/Actuators.h"
#include "acs/simulation/Aircraft.h"
#include "acs/simulation/Sensors.h"
#include "acs/simulation/Telemetry.h"
#include "acs/simulation/Wind.h"

#include <cstddef>

namespace acs::simulation {

struct SimulatorConfig {
  double dt_s{0.01};
  double sim_time_s{30.0};
  std::size_t max_steps{0};
  unsigned int seed{42U};
  WindConfig wind{};
  ActuatorConfig actuators{};
  ActuatorFaultConfig actuator_faults{};
};

class Simulator {
public:
  Simulator(const SimulatorConfig& sim_cfg,
            const Aircraft& aircraft,
            const acs::gnc::FlightController& controller_template,
            const SensorConfig& sensor_cfg);

  void run(const acs::gnc::ControllerTargets& targets, const std::string& telemetry_path);

private:
  SimulatorConfig sim_cfg_{};
  const Aircraft& aircraft_;
  acs::gnc::FlightController controller_;
  Sensors sensors_;
  WindModel wind_{};
  Actuators actuators_{};
};

}  // namespace acs::simulation
