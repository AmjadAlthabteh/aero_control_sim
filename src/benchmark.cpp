#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include "acs/gnc/FlightController.h"
#include "acs/math/Constants.h"
#include "acs/profiling/PerformanceProfiler.h"
#include "acs/simulation/Aircraft.h"
#include "acs/simulation/Simulator.h"

struct BenchmarkConfig {
  size_t num_simulations{10};
  size_t num_threads{std::thread::hardware_concurrency()};
  double sim_time_s{10.0};
  double dt_s{0.01};
  unsigned int seed{42U};
  bool verbose{false};
};

struct BenchmarkResult {
  size_t total_simulations{0};
  double total_time_s{0.0};
  double avg_sim_time_s{0.0};
  double throughput_sims_per_sec{0.0};
  double total_sim_time_simulated{0.0};
  double realtime_factor{0.0};
};

void run_single_simulation(const acs::simulation::AircraftConfig& aircraft_cfg,
                           const acs::gnc::ControllerConfig& ctrl_cfg,
                           const acs::simulation::SensorConfig& sensor_cfg,
                           const BenchmarkConfig& bench_cfg,
                           size_t sim_index) {
  acs::simulation::Aircraft aircraft(aircraft_cfg);
  acs::gnc::FlightController controller(ctrl_cfg);

  acs::simulation::SimulatorConfig sim_cfg{};
  sim_cfg.dt_s = bench_cfg.dt_s;
  sim_cfg.sim_time_s = bench_cfg.sim_time_s;
  sim_cfg.seed = bench_cfg.seed + static_cast<unsigned int>(sim_index);

  acs::simulation::Simulator sim(sim_cfg, aircraft, controller, sensor_cfg);

  using acs::math::kDeg2Rad;
  acs::gnc::ControllerTargets targets{};
  targets.altitude_m = 60.0;
  targets.heading_rad = 20.0 * kDeg2Rad;
  targets.roll_rad = 0.0;
  targets.pitch_rad = 0.0;
  targets.airspeed_m_s = 17.0;

  // Run without telemetry output for pure performance testing
  sim.run(targets, "");
}

void run_thread_batch(size_t num_sims,
                      const acs::simulation::AircraftConfig& aircraft_cfg,
                      const acs::gnc::ControllerConfig& ctrl_cfg,
                      const acs::simulation::SensorConfig& sensor_cfg,
                      const BenchmarkConfig& bench_cfg,
                      size_t first_sim_index) {
  for (size_t i = 0; i < num_sims; ++i) {
    run_single_simulation(aircraft_cfg, ctrl_cfg, sensor_cfg, bench_cfg, first_sim_index + i);
  }
}

BenchmarkResult run_benchmark(const BenchmarkConfig& bench_cfg) {
  using acs::math::Matrix3;
  using acs::math::Vector3;

  std::cout << "\n=== Benchmark Configuration ===\n";
  std::cout << "Simulations: " << bench_cfg.num_simulations << "\n";
  std::cout << "Threads: " << bench_cfg.num_threads << "\n";
  std::cout << "Sim Time: " << bench_cfg.sim_time_s << "s per simulation\n";
  std::cout << "Timestep: " << bench_cfg.dt_s << "s\n";
  std::cout << "Steps per sim: " << static_cast<size_t>(bench_cfg.sim_time_s / bench_cfg.dt_s) << "\n\n";

  // Setup aircraft configuration
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

  std::cout << "Starting benchmark...\n";
  auto start_time = std::chrono::high_resolution_clock::now();

  if (bench_cfg.num_threads == 1) {
    // Single-threaded execution
    for (size_t i = 0; i < bench_cfg.num_simulations; ++i) {
      if (bench_cfg.verbose && (i + 1) % 10 == 0) {
        std::cout << "  Completed " << (i + 1) << "/" << bench_cfg.num_simulations << "\n";
      }
      run_single_simulation(aircraft_cfg, ctrl_cfg, sensor_cfg, bench_cfg, i);
    }
  } else {
    // Multi-threaded execution
    std::vector<std::thread> threads;
    size_t sims_per_thread = bench_cfg.num_simulations / bench_cfg.num_threads;
    size_t remainder = bench_cfg.num_simulations % bench_cfg.num_threads;
    size_t first_sim_index = 0;

    for (size_t t = 0; t < bench_cfg.num_threads; ++t) {
      size_t sims_for_this_thread = sims_per_thread + (t < remainder ? 1 : 0);
      threads.emplace_back(run_thread_batch, sims_for_this_thread,
                          std::ref(aircraft_cfg), std::ref(ctrl_cfg),
                          std::ref(sensor_cfg), std::ref(bench_cfg), first_sim_index);
      first_sim_index += sims_for_this_thread;
    }

    for (auto& thread : threads) {
      thread.join();
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time).count();

  BenchmarkResult result;
  result.total_simulations = bench_cfg.num_simulations;
  result.total_time_s = duration / 1000.0;
  result.avg_sim_time_s = result.total_time_s / result.total_simulations;
  result.throughput_sims_per_sec = result.total_simulations / result.total_time_s;
  result.total_sim_time_simulated = bench_cfg.num_simulations * bench_cfg.sim_time_s;
  result.realtime_factor = result.total_sim_time_simulated / result.total_time_s;

  return result;
}

void print_benchmark_results(const BenchmarkResult& result) {
  std::cout << "\n=== Benchmark Results ===\n";
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Total time: " << result.total_time_s << "s\n";
  std::cout << "Avg time per sim: " << result.avg_sim_time_s << "s\n";
  std::cout << "Throughput: " << result.throughput_sims_per_sec << " sims/sec\n";
  std::cout << "Total simulated time: " << result.total_sim_time_simulated << "s\n";
  std::cout << "Realtime factor: " << result.realtime_factor << "x\n";
  std::cout << "========================\n\n";
}

int main(int argc, char* argv[]) {
  BenchmarkConfig config;

  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--sims" && i + 1 < argc) {
      config.num_simulations = std::stoul(argv[++i]);
    } else if (arg == "--threads" && i + 1 < argc) {
      config.num_threads = std::stoul(argv[++i]);
    } else if (arg == "--time" && i + 1 < argc) {
      config.sim_time_s = std::stod(argv[++i]);
    } else if (arg == "--dt" && i + 1 < argc) {
      config.dt_s = std::stod(argv[++i]);
    } else if (arg == "--seed" && i + 1 < argc) {
      config.seed = static_cast<unsigned int>(std::stoul(argv[++i]));
    } else if (arg == "--verbose" || arg == "-v") {
      config.verbose = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [options]\n";
      std::cout << "Options:\n";
      std::cout << "  --sims N      Number of simulations to run (default: 10)\n";
      std::cout << "  --threads N   Number of threads to use (default: hardware_concurrency)\n";
      std::cout << "  --time T      Simulation time in seconds (default: 10.0)\n";
      std::cout << "  --dt T        Simulation timestep in seconds (default: 0.01)\n";
      std::cout << "  --seed N      Base RNG seed, incremented per simulation (default: 42)\n";
      std::cout << "  --verbose     Show progress during benchmark\n";
      std::cout << "  --help        Show this help message\n";
      return 0;
    }
  }

  if (config.num_simulations == 0) {
    std::cerr << "Error: --sims must be greater than zero\n";
    return 1;
  }
  if (config.num_threads == 0) {
    config.num_threads = 1;
  }
  if (config.num_threads > config.num_simulations) {
    config.num_threads = config.num_simulations;
  }
  if (config.sim_time_s <= 0.0) {
    std::cerr << "Error: --time must be greater than zero\n";
    return 1;
  }
  if (config.dt_s <= 0.0 || config.dt_s > config.sim_time_s) {
    std::cerr << "Error: --dt must be greater than zero and no larger than --time\n";
    return 1;
  }

  // Run single-threaded benchmark
  std::cout << "\n>>> SINGLE-THREADED BENCHMARK <<<\n";
  BenchmarkConfig single_config = config;
  single_config.num_threads = 1;
  auto single_result = run_benchmark(single_config);
  print_benchmark_results(single_result);

  // Run multi-threaded benchmark
  std::cout << "\n>>> MULTI-THREADED BENCHMARK <<<\n";
  auto multi_result = run_benchmark(config);
  print_benchmark_results(multi_result);

  // Compare results
  std::cout << "=== Performance Comparison ===\n";
  double speedup = single_result.total_time_s / multi_result.total_time_s;
  double efficiency = speedup / config.num_threads * 100.0;
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Speedup: " << speedup << "x\n";
  std::cout << "Parallel efficiency: " << efficiency << "%\n";
  std::cout << "==============================\n";

  return 0;
}
