#include <cmath>
#include <iostream>
<<<<<<< HEAD
#include <string>

#include "acs/gnc/FlightController.h"
#include "acs/simulation/Aircraft.h"
#include "acs/simulation/Simulator.h"

static void print_help(const char* argv0) {
  std::cout << "Usage: " << argv0 << " [options]\n\n";
  std::cout << "Options:\n";
  std::cout << "  --dt S                 Timestep (default 0.01)\n";
  std::cout << "  --time S               Simulation time (default 35)\n";
  std::cout << "  --telemetry PATH       Telemetry output path (default telemetry.csv)\n";
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
  std::cout << "  --help                 Show this help\n";
}

static bool parse_double(const char* s, double& out) {
  try {
    out = std::stod(s);
    return true;
  } catch (...) {
    return false;
  }
}

static bool parse_uint(const char* s, unsigned int& out) {
  try {
    const auto v = std::stoul(s);
    out = static_cast<unsigned int>(v);
    return true;
  } catch (...) {
    return false;
  }
}

int main(int argc, char* argv[]) {
  using acs::math::Matrix3;
  using acs::math::Vector3;
  constexpr double kPi = 3.14159265358979323846;

  double dt_s = 0.01;
  double sim_time_s = 35.0;
  unsigned int seed = 42U;
  std::string telemetry_path = "telemetry.csv";

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

  acs::simulation::Aircraft aircraft(aircraft_cfg);

  acs::gnc::ControllerConfig ctrl_cfg{};
  ctrl_cfg.surface_limit_rad = aircraft_cfg.aero.aileron_limit_rad;
  acs::gnc::FlightController controller(ctrl_cfg);

  acs::simulation::SensorConfig sensor_cfg{};

  acs::simulation::SimulatorConfig sim_cfg{};
  sim_cfg.actuators.surface_limit_rad = aircraft_cfg.aero.aileron_limit_rad;

  // Parse options
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_help(argv[0]);
      return 0;
    } else if (arg == "--dt" && i + 1 < argc) {
      (void)parse_double(argv[++i], dt_s);
    } else if (arg == "--time" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_time_s);
    } else if (arg == "--telemetry" && i + 1 < argc) {
      telemetry_path = argv[++i];
    } else if (arg == "--seed" && i + 1 < argc) {
      (void)parse_uint(argv[++i], seed);
    } else if (arg == "--wind" && i + 3 < argc) {
      (void)parse_double(argv[++i], sim_cfg.wind.mean_ned_m_s.x);
      (void)parse_double(argv[++i], sim_cfg.wind.mean_ned_m_s.y);
      (void)parse_double(argv[++i], sim_cfg.wind.mean_ned_m_s.z);
    } else if (arg == "--gust-std" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_cfg.wind.gust_std_dev_m_s);
    } else if (arg == "--gust-tau" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_cfg.wind.gust_tau_s);
    } else if (arg == "--surface-tau" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_cfg.actuators.surface_tau_s);
    } else if (arg == "--surface-rate" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_cfg.actuators.surface_rate_limit_rad_s);
    } else if (arg == "--throttle-tau" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_cfg.actuators.throttle_tau_s);
    } else if (arg == "--throttle-rate" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_cfg.actuators.throttle_rate_limit_per_s);
    } else if (arg == "--stuck-aileron-time" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_cfg.actuator_faults.aileron_stuck_time_s);
    } else if (arg == "--stuck-elevator-time" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_cfg.actuator_faults.elevator_stuck_time_s);
    } else if (arg == "--stuck-rudder-time" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_cfg.actuator_faults.rudder_stuck_time_s);
    } else if (arg == "--stuck-throttle-time" && i + 1 < argc) {
      (void)parse_double(argv[++i], sim_cfg.actuator_faults.throttle_stuck_time_s);
    } else if (arg == "--gps-dropout-prob" && i + 1 < argc) {
      (void)parse_double(argv[++i], sensor_cfg.gps_dropout_prob);
    } else if (arg == "--baro-dropout-prob" && i + 1 < argc) {
      (void)parse_double(argv[++i], sensor_cfg.baro_dropout_prob);
    } else if (arg == "--accel-bias" && i + 3 < argc) {
      (void)parse_double(argv[++i], sensor_cfg.accel_bias_m_s2.x);
      (void)parse_double(argv[++i], sensor_cfg.accel_bias_m_s2.y);
      (void)parse_double(argv[++i], sensor_cfg.accel_bias_m_s2.z);
    } else if (arg == "--gyro-bias" && i + 3 < argc) {
      (void)parse_double(argv[++i], sensor_cfg.gyro_bias_rad_s.x);
      (void)parse_double(argv[++i], sensor_cfg.gyro_bias_rad_s.y);
      (void)parse_double(argv[++i], sensor_cfg.gyro_bias_rad_s.z);
    } else if (arg == "--accel-bias-rw" && i + 1 < argc) {
      (void)parse_double(argv[++i], sensor_cfg.accel_bias_rw_std_m_s2_sqrt_s);
    } else if (arg == "--gyro-bias-rw" && i + 1 < argc) {
      (void)parse_double(argv[++i], sensor_cfg.gyro_bias_rw_std_rad_s_sqrt_s);
    } else if (arg == "--hold-last-on-dropout" && i + 1 < argc) {
      double v = 1.0;
      (void)parse_double(argv[++i], v);
      sensor_cfg.hold_last_on_dropout = (v != 0.0);
    } else {
      std::cout << "Unknown option: " << arg << "\n";
      std::cout << "Use --help for usage.\n";
      return 1;
    }
  }

  sim_cfg.dt_s = dt_s;
  sim_cfg.sim_time_s = sim_time_s;
  sim_cfg.seed = seed;

  acs::simulation::Simulator sim(sim_cfg, aircraft, controller, sensor_cfg);

  acs::gnc::ControllerTargets targets{};
  targets.altitude_m = 60.0;
  targets.heading_rad = 20.0 * kPi / 180.0;
  targets.roll_rad = 0.0;
  targets.pitch_rad = 0.0;
  targets.airspeed_m_s = 17.0;

  sim.run(targets, telemetry_path);

  std::cout << "done, wrote " << telemetry_path << "\n";
  return 0;
=======

#include "aero/cli.hpp"
#include "aero/core.hpp"
#include "aero/csv.hpp"
#include "aero/models.hpp"
#include "aero/pid.hpp"
#include "aero/rk4.hpp"

namespace
{
constexpr double deg2rad(double deg) { return deg * (3.14159265358979323846 / 180.0); }
} // namespace

int main(int argc, char** argv)
{
    bool ok = true;
    const auto args = aero::parseArgs(argc, argv, ok);
    if (!ok || args.help)
    {
        std::cout << "aero_control_sim\n\n"
                     "Flags:\n"
                     "  --dt <seconds>         Timestep (default 0.01)\n"
                     "  --duration <seconds>   Sim duration (default 20)\n"
                     "  --step-alt <meters>    Altitude step command (default 100)\n"
                     "  --step-time <seconds>  Step time (default 1)\n"
                     "  --out <path>           Output CSV (default out.csv)\n";
        return ok ? 0 : 2;
    }

    aero::SimConfig cfg{};
    cfg.dt_s = args.dt_s;
    cfg.duration_s = args.duration_s;

    const auto plant = aero::makeToyLongitudinalModel();

    aero::Vec<4> x{};

    aero::PidConfig altCfg{};
    altCfg.kp = 0.002;
    altCfg.ki = 0.00015;
    altCfg.out_min = -deg2rad(20.0);
    altCfg.out_max = deg2rad(20.0);
    altCfg.i_min = -deg2rad(5.0);
    altCfg.i_max = deg2rad(5.0);
    altCfg.anti_windup = true;
    aero::Pid altPid(altCfg);

    aero::PidConfig pitchCfg{};
    pitchCfg.kp = 3.0;
    pitchCfg.ki = 0.8;
    pitchCfg.kd = 0.12;
    pitchCfg.out_min = -deg2rad(25.0);
    pitchCfg.out_max = deg2rad(25.0);
    pitchCfg.i_min = -deg2rad(10.0);
    pitchCfg.i_max = deg2rad(10.0);
    pitchCfg.d_filter_tau_s = 0.05;
    pitchCfg.anti_windup = true;
    aero::Pid pitchPid(pitchCfg);

    aero::CsvWriter csv(args.out_csv);
    if (!csv.ok())
    {
        std::cerr << "Failed to open " << args.out_csv << " for writing\n";
        return 1;
    }

    csv.writeHeader({"t_s", "h_m", "w_mps", "theta_rad", "q_radps", "h_cmd_m", "theta_cmd_rad", "elev_cmd_rad"});

    const double step_altitude_m = args.step_altitude_m;
    const double step_time_s = args.step_time_s;

    for (double t = 0.0; t <= cfg.duration_s; t += cfg.dt_s)
    {
        const double h_cmd = (t >= step_time_s) ? step_altitude_m : 0.0;
        const double h = x[0];
        const double theta = x[2];

        const double theta_cmd = altPid.update(h_cmd - h, cfg.dt_s);
        const double elev_cmd = pitchPid.update(theta_cmd - theta, cfg.dt_s);

        aero::Vec<1> u{};
        u[0] = elev_cmd;

        x = aero::rk4Step<4>(
            x,
            t,
            cfg.dt_s,
            [&](double /*time*/, const aero::Vec<4>& xs) { return plant.xdot(xs, u); });

        csv.writeRow({t, x[0], x[1], x[2], x[3], h_cmd, theta_cmd, elev_cmd});
    }

    std::cout << "Wrote " << args.out_csv << "\n";
    return 0;
>>>>>>> ff9e5a64303f6786744c89cf3ac13bfab0040f04
}
