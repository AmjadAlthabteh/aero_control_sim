#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "acs/config/ConfigLoader.h"
#include "acs/gnc/FlightController.h"
#include "acs/math/Constants.h"
#include "acs/simulation/Aircraft.h"
#include "acs/simulation/Simulator.h"

static void print_help(const char* argv0) {
  std::cout << "Usage: " << argv0 << " [options]\n\n";
  std::cout << "Options:\n";
  std::cout << "  --config PATH          Load simulation settings from JSON config\n";
  std::cout << "  --dt S                 Timestep (default 0.01)\n";
  std::cout << "  --time S               Simulation time (default 35)\n";
  std::cout << "  --max-steps N          Cap integration steps (default 0 = disabled)\n";
  std::cout << "  --telemetry PATH       Telemetry output path (default telemetry.csv)\n";
  std::cout << "  --no-telemetry         Disable telemetry logging\n";
  std::cout << "  --seed N               RNG seed for deterministic runs (default 42)\n";
  std::cout << "  --wind N E D           Mean wind in NED frame, m/s (default 0 0 0)\n";
  std::cout << "  --gust-std S           1-sigma gust per axis, m/s (default 0)\n";
  std::cout << "  --gust-tau S           Gust correlation time constant, s (default 2)\n";
  std::cout << "  --surface-tau S        Surface actuator time constant, s (default 0)\n";
  std::cout << "  --surface-rate S       Surface rate limit, rad/s (default inf)\n";
  std::cout << "  --throttle-tau S       Throttle actuator time constant, s (default 0)\n";
  std::cout << "  --throttle-rate S      Throttle rate limit, 1/s (default inf)\n";
  std::cout << "  --stuck-aileron-time S Aileron stuck fault latch time (default -1)\n";
  std::cout << "  --stuck-elevator-time S Elevator stuck fault latch time (default -1)\n";
  std::cout << "  --stuck-rudder-time S  Rudder stuck fault latch time (default -1)\n";
  std::cout << "  --stuck-throttle-time S Throttle stuck fault latch time (default -1)\n";
  std::cout << "  --gps-dropout-prob P   GPS dropout probability per sample (default 0)\n";
  std::cout << "  --baro-dropout-prob P  Baro dropout probability per sample (default 0)\n";
  std::cout << "  --accel-bias X Y Z     IMU accel bias in body frame, m/s^2 (default 0)\n";
  std::cout << "  --gyro-bias X Y Z      IMU gyro bias in body frame, rad/s (default 0)\n";
  std::cout << "  --accel-bias-rw S      IMU accel bias RW 1-sigma, m/s^2/sqrt(s) (default 0)\n";
  std::cout << "  --gyro-bias-rw S       IMU gyro bias RW 1-sigma, rad/s/sqrt(s) (default 0)\n";
  std::cout << "  --hold-last-on-dropout 0|1 (default 1)\n";
  std::cout << "  --target-alt M         Altitude hold target (default 60)\n";
  std::cout << "  --target-heading-deg D Heading hold target, deg (default 20)\n";
  std::cout << "  --target-airspeed MPS  Airspeed target, m/s (default 17)\n";
  std::cout << "  --target-roll-deg D    Roll trim bias, deg (default 0)\n";
  std::cout << "  --target-pitch-deg D   Pitch trim bias, deg (default 0)\n";
  std::cout << "  --help                 Show this help\n";
}

static bool parse_double(const char* s, double& out, const char* param_name) {
  try {
    out = std::stod(s);
    return true;
  } catch (...) {
    std::cerr << "Error: Invalid value '" << s << "' for parameter " << param_name << "\n";
    return false;
  }
}

static bool parse_uint(const char* s, unsigned int& out, const char* param_name) {
  try {
    const auto v = std::stoul(s);
    if (v > static_cast<unsigned long>(std::numeric_limits<unsigned int>::max())) {
      std::cerr << "Error: Value '" << s << "' for parameter " << param_name << " is out of range\n";
      return false;
    }
    out = static_cast<unsigned int>(v);
    return true;
  } catch (...) {
    std::cerr << "Error: Invalid value '" << s << "' for parameter " << param_name << "\n";
    return false;
  }
}

static bool parse_size(const char* s, std::size_t& out, const char* param_name) {
  try {
    out = static_cast<std::size_t>(std::stoull(s));
    return true;
  } catch (...) {
    std::cerr << "Error: Invalid value '" << s << "' for parameter " << param_name << "\n";
    return false;
  }
}

int main(int argc, char* argv[]) {
  using acs::math::kDeg2Rad;
  using acs::math::Matrix3;
  using acs::math::Vector3;

  double dt_s = 0.01;
  double sim_time_s = 35.0;
  unsigned int seed = 42U;
  std::string telemetry_path = "telemetry.csv";

  acs::gnc::ControllerTargets targets{};
  targets.altitude_m = 60.0;
  targets.heading_rad = 20.0 * kDeg2Rad;
  targets.roll_rad = 0.0;
  targets.pitch_rad = 0.0;
  targets.airspeed_m_s = 17.0;
  acs::simulation::MissionProfile mission = acs::simulation::MissionProfile::hold(targets);
  bool mission_from_config = false;

  // small fixed-wing-ish airframe
  acs::simulation::AircraftConfig aircraft_cfg{};
  aircraft_cfg.rigid_body.mass_kg = 2.0;
  aircraft_cfg.rigid_body.inertia_kg_m2 = Matrix3::from_rows(
      Vector3(0.06, 0.0, 0.0),
      Vector3(0.0, 0.08, 0.0),
      Vector3(0.0, 0.0, 0.12));
  aircraft_cfg.rigid_body.inertia_inv = aircraft_cfg.rigid_body.inertia_kg_m2.inverse();

  aircraft_cfg.aero.s_ref_m2 = 0.35;
  aircraft_cfg.aero.b_ref_m = 1.5;
  aircraft_cfg.aero.c_ref_m = 0.25;
  aircraft_cfg.aero.max_thrust_n = 22.0;

  acs::gnc::ControllerConfig ctrl_cfg{};
  ctrl_cfg.surface_limit_rad = aircraft_cfg.aero.aileron_limit_rad;

  acs::simulation::SensorConfig sensor_cfg{};

  acs::simulation::SimulatorConfig sim_cfg{};
  sim_cfg.actuators.surface_limit_rad = aircraft_cfg.aero.aileron_limit_rad;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg != "--config") continue;
    if (i + 1 >= argc) {
      std::cerr << "Error: --config requires a path\n";
      return 1;
    }

    try {
      const auto params = acs::config::ConfigLoader::load_from_file(argv[++i]);
      dt_s = params.dt_s;
      sim_time_s = params.sim_time_s;
      seed = params.seed;
      telemetry_path = params.enable_telemetry ? params.telemetry_path : std::string{};
      targets = params.targets;
      aircraft_cfg = params.aircraft_cfg;
      ctrl_cfg = params.ctrl_cfg;
      sensor_cfg = params.sensor_cfg;
      sim_cfg = params.sim_cfg;
      mission = params.mission;
      mission_from_config = params.mission.size() > 1;
    } catch (const std::exception& e) {
      std::cerr << "Error: Failed to load config: " << e.what() << "\n";
      return 1;
    }
  }

  // Parse options
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_help(argv[0]);
      return 0;
    } else if (arg == "--config" && i + 1 < argc) {
      ++i;
    } else if (arg == "--dt" && i + 1 < argc) {
      if (!parse_double(argv[++i], dt_s, "--dt")) return 1;
    } else if (arg == "--time" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_time_s, "--time")) return 1;
    } else if (arg == "--max-steps" && i + 1 < argc) {
      if (!parse_size(argv[++i], sim_cfg.max_steps, "--max-steps")) return 1;
    } else if (arg == "--telemetry" && i + 1 < argc) {
      telemetry_path = argv[++i];
    } else if (arg == "--no-telemetry") {
      telemetry_path.clear();
    } else if (arg == "--seed" && i + 1 < argc) {
      if (!parse_uint(argv[++i], seed, "--seed")) return 1;
    } else if (arg == "--wind" && i + 3 < argc) {
      if (!parse_double(argv[++i], sim_cfg.wind.mean_ned_m_s.x, "--wind N")) return 1;
      if (!parse_double(argv[++i], sim_cfg.wind.mean_ned_m_s.y, "--wind E")) return 1;
      if (!parse_double(argv[++i], sim_cfg.wind.mean_ned_m_s.z, "--wind D")) return 1;
    } else if (arg == "--gust-std" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_cfg.wind.gust_std_dev_m_s, "--gust-std")) return 1;
    } else if (arg == "--gust-tau" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_cfg.wind.gust_tau_s, "--gust-tau")) return 1;
    } else if (arg == "--surface-tau" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_cfg.actuators.surface_tau_s, "--surface-tau")) return 1;
    } else if (arg == "--surface-rate" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_cfg.actuators.surface_rate_limit_rad_s, "--surface-rate")) return 1;
    } else if (arg == "--throttle-tau" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_cfg.actuators.throttle_tau_s, "--throttle-tau")) return 1;
    } else if (arg == "--throttle-rate" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_cfg.actuators.throttle_rate_limit_per_s, "--throttle-rate")) return 1;
    } else if (arg == "--stuck-aileron-time" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_cfg.actuator_faults.aileron_stuck_time_s, "--stuck-aileron-time")) return 1;
    } else if (arg == "--stuck-elevator-time" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_cfg.actuator_faults.elevator_stuck_time_s, "--stuck-elevator-time")) return 1;
    } else if (arg == "--stuck-rudder-time" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_cfg.actuator_faults.rudder_stuck_time_s, "--stuck-rudder-time")) return 1;
    } else if (arg == "--stuck-throttle-time" && i + 1 < argc) {
      if (!parse_double(argv[++i], sim_cfg.actuator_faults.throttle_stuck_time_s, "--stuck-throttle-time")) return 1;
    } else if (arg == "--gps-dropout-prob" && i + 1 < argc) {
      if (!parse_double(argv[++i], sensor_cfg.gps_dropout_prob, "--gps-dropout-prob")) return 1;
    } else if (arg == "--baro-dropout-prob" && i + 1 < argc) {
      if (!parse_double(argv[++i], sensor_cfg.baro_dropout_prob, "--baro-dropout-prob")) return 1;
    } else if (arg == "--accel-bias" && i + 3 < argc) {
      if (!parse_double(argv[++i], sensor_cfg.accel_bias_m_s2.x, "--accel-bias X")) return 1;
      if (!parse_double(argv[++i], sensor_cfg.accel_bias_m_s2.y, "--accel-bias Y")) return 1;
      if (!parse_double(argv[++i], sensor_cfg.accel_bias_m_s2.z, "--accel-bias Z")) return 1;
    } else if (arg == "--gyro-bias" && i + 3 < argc) {
      if (!parse_double(argv[++i], sensor_cfg.gyro_bias_rad_s.x, "--gyro-bias X")) return 1;
      if (!parse_double(argv[++i], sensor_cfg.gyro_bias_rad_s.y, "--gyro-bias Y")) return 1;
      if (!parse_double(argv[++i], sensor_cfg.gyro_bias_rad_s.z, "--gyro-bias Z")) return 1;
    } else if (arg == "--accel-bias-rw" && i + 1 < argc) {
      if (!parse_double(argv[++i], sensor_cfg.accel_bias_rw_std_m_s2_sqrt_s, "--accel-bias-rw")) return 1;
    } else if (arg == "--gyro-bias-rw" && i + 1 < argc) {
      if (!parse_double(argv[++i], sensor_cfg.gyro_bias_rw_std_rad_s_sqrt_s, "--gyro-bias-rw")) return 1;
    } else if (arg == "--hold-last-on-dropout" && i + 1 < argc) {
      double v = 1.0;
      if (!parse_double(argv[++i], v, "--hold-last-on-dropout")) return 1;
      sensor_cfg.hold_last_on_dropout = (v != 0.0);
    } else if (arg == "--target-alt" && i + 1 < argc) {
      if (!parse_double(argv[++i], targets.altitude_m, "--target-alt")) return 1;
    } else if (arg == "--target-heading-deg" && i + 1 < argc) {
      double deg = 0.0;
      if (!parse_double(argv[++i], deg, "--target-heading-deg")) return 1;
      targets.heading_rad = deg * kDeg2Rad;
    } else if (arg == "--target-airspeed" && i + 1 < argc) {
      if (!parse_double(argv[++i], targets.airspeed_m_s, "--target-airspeed")) return 1;
    } else if (arg == "--target-roll-deg" && i + 1 < argc) {
      double deg = 0.0;
      if (!parse_double(argv[++i], deg, "--target-roll-deg")) return 1;
      targets.roll_rad = deg * kDeg2Rad;
    } else if (arg == "--target-pitch-deg" && i + 1 < argc) {
      double deg = 0.0;
      if (!parse_double(argv[++i], deg, "--target-pitch-deg")) return 1;
      targets.pitch_rad = deg * kDeg2Rad;
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      std::cerr << "Use --help for usage.\n";
      return 1;
    }
  }

  // Validate simulation parameters
  if (dt_s <= 0.0 || dt_s > 1.0) {
    std::cerr << "Error: --dt must be in range (0, 1.0], got " << dt_s << "\n";
    return 1;
  }
  if (sim_time_s <= 0.0 || sim_time_s > 10000.0) {
    std::cerr << "Error: --time must be in range (0, 10000], got " << sim_time_s << "\n";
    return 1;
  }
  if (sim_cfg.wind.gust_std_dev_m_s < 0.0) {
    std::cerr << "Error: --gust-std must be non-negative, got " << sim_cfg.wind.gust_std_dev_m_s << "\n";
    return 1;
  }
  if (sim_cfg.wind.gust_tau_s <= 0.0) {
    std::cerr << "Error: --gust-tau must be positive, got " << sim_cfg.wind.gust_tau_s << "\n";
    return 1;
  }
  if (sensor_cfg.gps_dropout_prob < 0.0 || sensor_cfg.gps_dropout_prob > 1.0) {
    std::cerr << "Error: --gps-dropout-prob must be in [0, 1], got " << sensor_cfg.gps_dropout_prob << "\n";
    return 1;
  }
  if (sensor_cfg.baro_dropout_prob < 0.0 || sensor_cfg.baro_dropout_prob > 1.0) {
    std::cerr << "Error: --baro-dropout-prob must be in [0, 1], got " << sensor_cfg.baro_dropout_prob << "\n";
    return 1;
  }
  if (targets.altitude_m < -1000.0 || targets.altitude_m > 100000.0) {
    std::cerr << "Error: --target-alt must be in range [-1000, 100000], got " << targets.altitude_m << "\n";
    return 1;
  }
  if (targets.airspeed_m_s <= 0.0 || targets.airspeed_m_s > 1000.0) {
    std::cerr << "Error: --target-airspeed must be in range (0, 1000], got " << targets.airspeed_m_s << "\n";
    return 1;
  }

  sim_cfg.dt_s = dt_s;
  sim_cfg.sim_time_s = sim_time_s;
  sim_cfg.seed = seed;
  if (!mission_from_config) {
    mission = acs::simulation::MissionProfile::hold(targets);
  }

  acs::simulation::Aircraft aircraft(aircraft_cfg);
  acs::gnc::FlightController controller(ctrl_cfg);
  acs::simulation::Simulator sim(sim_cfg, aircraft, controller, sensor_cfg);

  sim.run(mission, telemetry_path);

  std::cout << "done, wrote " << telemetry_path << "\n";
  return 0;
}
