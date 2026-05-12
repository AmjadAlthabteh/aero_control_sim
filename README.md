# aero_control_sim

Small, dependency-light aircraft control simulation playground.

## Goals

- Simulate a simple aircraft model with a fixed timestep.
- Implement common control blocks (PID, filters, saturations).
- Export data to CSV for plotting/analysis.

## Status

Work in progress.

## Build

```bash
cmake -S . -B build
cmake --build build --config Release
```

## Run

```bash
./build/Release/aero_control_sim
```

Writes `out.csv` in the repo root (time history of altitude/pitch/elevator).
