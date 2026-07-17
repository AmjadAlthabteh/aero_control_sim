# Operator Notes

## Startup Checklist

- Build in release mode before collecting performance numbers.
- Run the simulator once without custom flags to confirm telemetry output is created.
- Keep the terminal output from the first run for quick comparison against later changes.

## Telemetry Review

- Open `telemetry.csv` after a run and confirm the timestamp column increases steadily.
- Check target columns first when validating mission-profile behavior.
- Compare attitude and altitude trends before tuning controller gains.
