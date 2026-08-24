# aero-control-system-sim

`aero-control-system-sim` is a high-performance c++17 flight dynamics simulator with a 6dof rigid-body model, basic aerodynamics, sensor simulation, pid-based stabilization/hold modes, and comprehensive performance profiling and testing tools.

## features

### core simulation
- 6dof newton-euler rigid body dynamics (body-frame velocities, body rates)
- quaternion attitude propagation (with euler angles for telemetry)
- rk4 integrator with fixed timestep
- simple atmosphere + gravity models
- aerodynamic lift/drag/sideforce + moments with control surface effects
- wind + gust/turbulence model (ned frame)
- actuator dynamics: first-order lag + rate limiting, with stuck-surface fault injection
- imu/gps/barometer sensors with gaussian noise, bias random walk, and dropout simulation
- roll/pitch/heading/altitude hold pid controllers
- time-scheduled mission profiles with interpolated altitude, heading, airspeed, roll, and pitch targets
- csv telemetry logging

### performance & testing (new!)
- **performance profiler**: real-time timing metrics for all simulation components
  - frame timing, physics integration, control loop latency
  - min/max/avg statistics with detailed profiling reports
  - scoped profiling with automatic tracking
- **benchmark suite**: multi-threaded throughput testing
  - parallel simulation execution for performance analysis
  - speedup and efficiency metrics
  - configurable simulation count and thread pool size
- **statistics & analytics**: comprehensive telemetry analysis
  - statistical summaries (min/max/mean/median/std dev)
  - attitude, position, velocity, and control surface analysis
  - stability detection and steady-state error computation
- **unit testing framework**: lightweight testing infrastructure
  - tests for vector math, quaternions, matrices
  - physics model validation (atmosphere, gravity)
  - performance benchmarks for critical operations
- **performance monitor**: live dashboard visualization
  - real-time FPS and frame time monitoring
  - performance bar graphs and metrics export
  - csv export for historical analysis

## build

Use an out-of-source build directory so generated files stay separate from source files.

### linux/macos
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### windows (msvc)
```bash
cmake -S . -B build
cmake --build build --config Release
```

### build options
- `ACS_BUILD_TOOLS` controls the benchmark and analyzer executables.
- `AERO_CONTROL_SIM_BUILD_TESTS` controls the ctest test targets.

## executables

the build process creates several executables:

1. **aero-control-system-sim** - main flight simulator
   ```bash
   ./build/aero-control-system-sim
   ```
   writes `telemetry.csv` with full flight data and displays performance profiling report

2. **acs-benchmark** - performance benchmarking tool
   ```bash
   ./build/acs-benchmark --sims 50 --threads 4 --time 10.0
   ```
   runs parallel simulations and reports throughput metrics

3. **acs-analyzer** - telemetry analysis tool
   ```bash
   ./build/acs-analyzer telemetry.csv
   ```
   analyzes flight data and provides detailed statistics

4. **tests** - ctest-registered unit tests
   ```bash
   ctest --test-dir build -C Release --output-on-failure
   ```
   runs the unit tests under `tests/`

## coordinate frames and conventions
- navigation frame: **ned** (north-east-down), position in meters, velocity in m/s
- body frame: **frd** (forward-right-down), body velocities and angular rates
- quaternion `q_nb` rotates a body-frame vector into ned: `v_n = q_nb.rotate(v_b)`
- euler angles are 3-2-1 (yaw-pitch-roll) for logging only

## performance characteristics

typical performance metrics (example system):
- **simulation speed**: 1000-3000+ fps on modern hardware
- **realtime factor**: 100-300x realtime (can simulate 30s in < 0.3s)
- **throughput**: 100+ simulations/second (multi-threaded)
- **frame time**: 0.3-1.0 ms per simulation step
- **physics integration**: < 0.15 ms per step
- **control loop**: < 0.05 ms per update

## usage examples

### run simulation with profiling
```bash
./build/aero-control-system-sim
# outputs telemetry.csv and displays performance report
```

### run without telemetry output
```bash
./build/aero-control-system-sim --no-telemetry
```

### run with wind + actuator dynamics + deterministic seed
```bash
./build/aero-control-system-sim --seed 123 --wind 3.0 0.0 0.0 --gust-std 1.0 --gust-tau 2.0 --surface-tau 0.05 --surface-rate 3.0
```

### run with custom hold targets
```bash
./build/aero-control-system-sim --target-alt 120 --target-heading-deg -45 --target-airspeed 20
```

### run a scheduled mission from config
```json
{
  "targets": {"altitude_m": 60, "heading_deg": 20, "airspeed_m_s": 17},
  "mission": {
    "waypoints": [
      {"time_s": 0, "altitude_m": 60, "heading_deg": 20, "airspeed_m_s": 17},
      {"time_s": 8, "altitude_m": 85, "heading_deg": 45, "airspeed_m_s": 18.5},
      {"time_s": 18, "altitude_m": 70, "heading_deg": -20, "airspeed_m_s": 16.5}
    ]
  }
}
```

Mission waypoint fields inherit from the previous waypoint, so a waypoint can change only the target that matters. Headings interpolate along the shortest angular path, and telemetry records both the active target values and `mission_segment`.

### benchmark throughput
```bash
# single-threaded vs multi-threaded comparison
./build/acs-benchmark --sims 100 --threads 8

# quick benchmark with verbose output
./build/acs-benchmark --sims 20 --verbose
```

### analyze telemetry
```bash
./build/acs-analyzer telemetry.csv
# displays statistical analysis of flight data
```

### run tests
```bash
ctest --test-dir build -C Release --output-on-failure
```

Tests are enabled by default with `-DAERO_CONTROL_SIM_BUILD_TESTS=ON`. To run a focused check while iterating on analytics helpers:
```bash
ctest --test-dir build -C Release -R statistics_utils --output-on-failure
```

## notes
the aero model is intentionally lightweight (linear coefficients + simple control derivatives). it is good enough to demonstrate realistic couplings (aoa, sideslip, damping moments) and closed-loop stabilization without pulling in a full coefficient database.

## performance tips
- use release builds (`-DCMAKE_BUILD_TYPE=Release`) for maximum performance
- benchmark mode disables telemetry I/O for pure compute performance
- multi-threaded benchmarks scale nearly linearly up to physical core count
- profiling adds minimal overhead (< 1-2% typical)
