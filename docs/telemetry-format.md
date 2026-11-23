# Telemetry format

Each simulation run writes two files:

- `<name>.csv`: numeric samples containing only simulator-native channels.
- `<name>.csv.metadata.json`: the machine-readable schema, units, coordinate
  frames, and configured sampling rate for that CSV.

Consumers should use the metadata instead of inferring units or sampling rate
from a generic aircraft telemetry layout. In particular:

- `airspeed_mps` and all other `_mps` channels use metres per second.
- Navigation vectors use NED; body vectors use FRD.
- Only columns listed in `columns` were measured or computed by the simulator.
- `channel_groups` separates state, air-data, applied-control, command,
  validity, environmental, and reference-target fields. Analyses of actual
  vehicle behavior should exclude the `reference_targets` group.
- `reference_channels_synthesized` is `false`. An absent channel such as EGT,
  N2, hydraulic pressure, flap position, or yaw trim must remain unavailable.
  It must not be assigned a nominal value or included in channel-count or
  cross-channel-coupling statistics.
- A downstream tool may add derived or reference channels only if it labels
  them separately and excludes them from claims about native telemetry.

The CSV timestamp is the time of the logged post-integration state. The
metadata `sample_period_s` and `sample_rate_hz` describe the configured logging
cadence; consumers should still check timestamp differences for dropped or
nonuniform samples.
