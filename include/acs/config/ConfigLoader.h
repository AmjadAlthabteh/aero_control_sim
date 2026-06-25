#pragma once

#include "acs/config/JsonConfig.h"
#include "acs/gnc/FlightController.h"
#include "acs/math/Constants.h"
#include "acs/math/Vector3.h"
#include "acs/simulation/Aircraft.h"
#include "acs/simulation/Sensors.h"
#include "acs/simulation/Simulator.h"

#include <string>

namespace acs::config {

struct SimulationParameters {
  double dt_s{0.01};
  double sim_time_s{35.0};
  unsigned int seed{42};
  std::string telemetry_path{"telemetry.csv"};
  bool enable_telemetry{true};

  acs::gnc::ControllerTargets targets{};
  acs::simulation::AircraftConfig aircraft_cfg{};
  acs::gnc::ControllerConfig ctrl_cfg{};
  acs::simulation::SensorConfig sensor_cfg{};
  acs::simulation::SimulatorConfig sim_cfg{};
};

class ConfigLoader {
public:
  static SimulationParameters load_from_file(const std::string& path) {
    JsonValue root = JsonParser::parse_file(path);
    return load_from_json(root);
  }

  static SimulationParameters load_from_json(const JsonValue& root) {
    SimulationParameters params;

    // Load basic simulation parameters
    if (root.has("dt_s")) params.dt_s = root.get("dt_s").as_number();
    if (root.has("sim_time_s")) params.sim_time_s = root.get("sim_time_s").as_number();
    if (root.has("max_steps")) params.sim_cfg.max_steps = static_cast<size_t>(root.get("max_steps").as_number());
    if (root.has("seed")) params.seed = static_cast<unsigned int>(root.get("seed").as_number());
    if (root.has("telemetry_path")) params.telemetry_path = root.get("telemetry_path").as_string();
    if (root.has("enable_telemetry")) params.enable_telemetry = root.get("enable_telemetry").as_bool();

    // Load control targets
    if (root.has("targets")) {
      auto targets = root.get("targets");
      if (targets.has("altitude_m"))
        params.targets.altitude_m = targets.get("altitude_m").as_number();
      if (targets.has("heading_deg"))
        params.targets.heading_rad = targets.get("heading_deg").as_number() * acs::math::kDeg2Rad;
      if (targets.has("airspeed_m_s"))
        params.targets.airspeed_m_s = targets.get("airspeed_m_s").as_number();
      if (targets.has("roll_deg"))
        params.targets.roll_rad = targets.get("roll_deg").as_number() * acs::math::kDeg2Rad;
      if (targets.has("pitch_deg"))
        params.targets.pitch_rad = targets.get("pitch_deg").as_number() * acs::math::kDeg2Rad;
    }

    // Load aircraft configuration
    if (root.has("aircraft")) {
      auto aircraft = root.get("aircraft");

      // Rigid body
      if (aircraft.has("mass_kg"))
        params.aircraft_cfg.rigid_body.mass_kg = aircraft.get("mass_kg").as_number();

      if (aircraft.has("inertia_kg_m2")) {
        auto inertia = aircraft.get("inertia_kg_m2").as_array();
        if (inertia.size() == 3) {
          auto row0 = inertia[0].as_array();
          auto row1 = inertia[1].as_array();
          auto row2 = inertia[2].as_array();
          params.aircraft_cfg.rigid_body.inertia_kg_m2 = acs::math::Matrix3::from_rows(
              acs::math::Vector3(row0[0].as_number(), row0[1].as_number(), row0[2].as_number()),
              acs::math::Vector3(row1[0].as_number(), row1[1].as_number(), row1[2].as_number()),
              acs::math::Vector3(row2[0].as_number(), row2[1].as_number(), row2[2].as_number()));
          params.aircraft_cfg.rigid_body.inertia_inv = params.aircraft_cfg.rigid_body.inertia_kg_m2.inverse();
        }
      }

      // Aerodynamics
      if (aircraft.has("s_ref_m2"))
        params.aircraft_cfg.aero.s_ref_m2 = aircraft.get("s_ref_m2").as_number();
      if (aircraft.has("b_ref_m"))
        params.aircraft_cfg.aero.b_ref_m = aircraft.get("b_ref_m").as_number();
      if (aircraft.has("c_ref_m"))
        params.aircraft_cfg.aero.c_ref_m = aircraft.get("c_ref_m").as_number();
      if (aircraft.has("max_thrust_n"))
        params.aircraft_cfg.aero.max_thrust_n = aircraft.get("max_thrust_n").as_number();
    }

    // Load controller configuration
    if (root.has("controller")) {
      auto controller = root.get("controller");
      auto load_gains = [](const JsonValue& obj, acs::gnc::PID::Gains& gains) {
        if (obj.has("kp")) gains.kp = obj.get("kp").as_number();
        if (obj.has("ki")) gains.ki = obj.get("ki").as_number();
        if (obj.has("kd")) gains.kd = obj.get("kd").as_number();
        if (obj.has("kaw")) gains.kaw = obj.get("kaw").as_number();
        if (obj.has("tau_d")) gains.tau_d = obj.get("tau_d").as_number();
      };

      if (controller.has("roll_gains")) load_gains(controller.get("roll_gains"), params.ctrl_cfg.roll_gains);
      if (controller.has("pitch_gains")) load_gains(controller.get("pitch_gains"), params.ctrl_cfg.pitch_gains);
      if (controller.has("heading_gains")) load_gains(controller.get("heading_gains"), params.ctrl_cfg.heading_gains);
      if (controller.has("altitude_gains")) load_gains(controller.get("altitude_gains"), params.ctrl_cfg.altitude_gains);
      if (controller.has("integrator_limit"))
        params.ctrl_cfg.integrator_limit = controller.get("integrator_limit").as_number();
      if (controller.has("surface_limit_rad"))
        params.ctrl_cfg.surface_limit_rad = controller.get("surface_limit_rad").as_number();
      if (controller.has("throttle_limit"))
        params.ctrl_cfg.throttle_limit = controller.get("throttle_limit").as_number();
    }

    // Load wind configuration
    if (root.has("wind")) {
      auto wind = root.get("wind");
      if (wind.has("mean_ned_m_s")) {
        auto mean = wind.get("mean_ned_m_s").as_array();
        if (mean.size() == 3) {
          params.sim_cfg.wind.mean_ned_m_s = acs::math::Vector3(
              mean[0].as_number(), mean[1].as_number(), mean[2].as_number());
        }
      }
      if (wind.has("gust_std_dev_m_s"))
        params.sim_cfg.wind.gust_std_dev_m_s = wind.get("gust_std_dev_m_s").as_number();
      if (wind.has("gust_tau_s"))
        params.sim_cfg.wind.gust_tau_s = wind.get("gust_tau_s").as_number();
    }

    // Load actuator configuration
    if (root.has("actuators")) {
      auto act = root.get("actuators");
      if (act.has("surface_tau_s"))
        params.sim_cfg.actuators.surface_tau_s = act.get("surface_tau_s").as_number();
      if (act.has("surface_rate_limit_rad_s"))
        params.sim_cfg.actuators.surface_rate_limit_rad_s = act.get("surface_rate_limit_rad_s").as_number();
      if (act.has("throttle_tau_s"))
        params.sim_cfg.actuators.throttle_tau_s = act.get("throttle_tau_s").as_number();
      if (act.has("throttle_rate_limit_per_s"))
        params.sim_cfg.actuators.throttle_rate_limit_per_s = act.get("throttle_rate_limit_per_s").as_number();
      if (act.has("surface_limit_rad"))
        params.sim_cfg.actuators.surface_limit_rad = act.get("surface_limit_rad").as_number();
      if (act.has("surface_deadband_rad"))
        params.sim_cfg.actuators.surface_deadband_rad = act.get("surface_deadband_rad").as_number();
    }

    // Load actuator faults
    if (root.has("actuator_faults")) {
      auto faults = root.get("actuator_faults");
      if (faults.has("aileron_stuck_time_s"))
        params.sim_cfg.actuator_faults.aileron_stuck_time_s = faults.get("aileron_stuck_time_s").as_number();
      if (faults.has("elevator_stuck_time_s"))
        params.sim_cfg.actuator_faults.elevator_stuck_time_s = faults.get("elevator_stuck_time_s").as_number();
      if (faults.has("rudder_stuck_time_s"))
        params.sim_cfg.actuator_faults.rudder_stuck_time_s = faults.get("rudder_stuck_time_s").as_number();
      if (faults.has("throttle_stuck_time_s"))
        params.sim_cfg.actuator_faults.throttle_stuck_time_s = faults.get("throttle_stuck_time_s").as_number();
    }

    // Load sensor configuration
    if (root.has("sensors")) {
      auto sensors = root.get("sensors");
      if (sensors.has("gps_dropout_prob"))
        params.sensor_cfg.gps_dropout_prob = sensors.get("gps_dropout_prob").as_number();
      if (sensors.has("baro_dropout_prob"))
        params.sensor_cfg.baro_dropout_prob = sensors.get("baro_dropout_prob").as_number();
      if (sensors.has("hold_last_on_dropout"))
        params.sensor_cfg.hold_last_on_dropout = sensors.get("hold_last_on_dropout").as_bool();

      if (sensors.has("accel_bias_m_s2")) {
        auto bias = sensors.get("accel_bias_m_s2").as_array();
        if (bias.size() == 3) {
          params.sensor_cfg.accel_bias_m_s2 = acs::math::Vector3(
              bias[0].as_number(), bias[1].as_number(), bias[2].as_number());
        }
      }

      if (sensors.has("gyro_bias_rad_s")) {
        auto bias = sensors.get("gyro_bias_rad_s").as_array();
        if (bias.size() == 3) {
          params.sensor_cfg.gyro_bias_rad_s = acs::math::Vector3(
              bias[0].as_number(), bias[1].as_number(), bias[2].as_number());
        }
      }

      if (sensors.has("accel_bias_rw_std_m_s2_sqrt_s"))
        params.sensor_cfg.accel_bias_rw_std_m_s2_sqrt_s = sensors.get("accel_bias_rw_std_m_s2_sqrt_s").as_number();
      if (sensors.has("gyro_bias_rw_std_rad_s_sqrt_s"))
        params.sensor_cfg.gyro_bias_rw_std_rad_s_sqrt_s = sensors.get("gyro_bias_rw_std_rad_s_sqrt_s").as_number();
    }

    const bool controller_surface_limit_configured =
        root.has("controller") && root.get("controller").has("surface_limit_rad");
    const bool actuator_surface_limit_configured =
        root.has("actuators") && root.get("actuators").has("surface_limit_rad");

    // Set derived parameters
    params.sim_cfg.dt_s = params.dt_s;
    params.sim_cfg.sim_time_s = params.sim_time_s;
    params.sim_cfg.seed = params.seed;
    if (!actuator_surface_limit_configured) {
      params.sim_cfg.actuators.surface_limit_rad = params.aircraft_cfg.aero.aileron_limit_rad;
    }
    if (!controller_surface_limit_configured) {
      params.ctrl_cfg.surface_limit_rad = params.aircraft_cfg.aero.aileron_limit_rad;
    }

    return params;
  }

  static SimulationParameters get_default_config() {
    using acs::math::Matrix3;
    using acs::math::Vector3;
    using acs::math::kDeg2Rad;

    SimulationParameters params{};
    params.targets.altitude_m = 60.0;
    params.targets.heading_rad = 20.0 * kDeg2Rad;
    params.targets.airspeed_m_s = 17.0;

    params.aircraft_cfg.rigid_body.mass_kg = 2.0;
    params.aircraft_cfg.rigid_body.inertia_kg_m2 = Matrix3::from_rows(
        Vector3(0.06, 0.0, 0.0),
        Vector3(0.0, 0.08, 0.0),
        Vector3(0.0, 0.0, 0.12));
    params.aircraft_cfg.rigid_body.inertia_inv = params.aircraft_cfg.rigid_body.inertia_kg_m2.inverse();

    params.aircraft_cfg.aero.s_ref_m2 = 0.35;
    params.aircraft_cfg.aero.b_ref_m = 1.5;
    params.aircraft_cfg.aero.c_ref_m = 0.25;
    params.aircraft_cfg.aero.max_thrust_n = 22.0;

    params.sim_cfg.dt_s = params.dt_s;
    params.sim_cfg.sim_time_s = params.sim_time_s;
    params.sim_cfg.seed = params.seed;
    params.sim_cfg.actuators.surface_limit_rad = params.aircraft_cfg.aero.aileron_limit_rad;
    params.ctrl_cfg.surface_limit_rad = params.aircraft_cfg.aero.aileron_limit_rad;

    return params;
  }
};

} // namespace acs::config
