# Operator Notes

## Startup Checklist

- Build in release mode before collecting performance numbers.
- Run the simulator once without custom flags to confirm telemetry output is created.
- Keep the terminal output from the first run for quick comparison against later changes.
- Save the exact command line used for any run that produces a reference telemetry file.

## Telemetry Review

- Open `telemetry.csv` after a run and confirm the timestamp column increases steadily.
- Check target columns first when validating mission-profile behavior.
- Compare attitude and altitude trends before tuning controller gains.
- Confirm the configured telemetry path before overwriting a prior run.

## Wind Scenario Notes

- Start with a constant wind vector before adding gust noise.
- Increase `--gust-std` gradually so controller response changes are easy to isolate.
- Reuse the same `--seed` when comparing wind model changes.
- Keep mean wind and gust settings in the run notes for replayability.

## Actuator Checks

- Confirm surface rate limits are documented with the scenario under test.
- Test stuck-surface faults one axis at a time.
- Review control commands and achieved deflections together when diagnosing lag.
- Include actuator deadband values when comparing small control inputs.

## Mission Profiles

- Keep waypoint times strictly increasing.
- Leave unchanged waypoint fields omitted so inherited targets stay obvious.
- Use heading changes smaller than 180 degrees when validating interpolation manually.
- Verify the final waypoint returns the vehicle to an expected steady target.

## Benchmark Runs

- Disable telemetry output when measuring pure simulation throughput.
- Record the simulation count, thread count, and build type with each benchmark.
- Repeat short benchmark runs before comparing small performance differences.

## Focused Test Runs

- Use `ctest -R` to rerun the smallest relevant test group while iterating.
- Run the full suite before pushing behavior changes.
- Keep failing telemetry samples when they explain an analyzer or controller issue.
