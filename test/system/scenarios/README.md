# Validation scenarios

One file per validation case in the register (SDP §3.6). CMake adds one
system test per file, `test_scenario_<file name>`, which runs it in its own
headless Gazebo on the empty course (`test_gnc_state_estimation_scenario`).

```yaml
id: VAL-005                 # register row
title: Full-throttle launch at mu 0.5
tire_mu: 0.5                # tyre friction (ground is 1.0; Gazebo uses the lower)
steps:                      # open-loop commands through steering_relay, 20 Hz
  - duration: 4.0           # s, sim time
    speed: 3.0              # m/s at the start of the step (negative = reverse)
    speed_end: 3.0          # optional: ramp linearly to this by the end
    steer: 0.0              # fraction of full lock, -1 (right) to +1 (left)
    expect_slip: true       # optional: a slip event must be counted in this step
checks:                     # every check is optional
  max_position_error: 0.3   # m, against truth, whole run
  final_position_error: 0.2 # m
  max_yaw_error_deg: 2.0
  min_consistent_fraction: 0.95   # true error inside the reported 3-sigma ellipse
  slip_events: {max: 0}     # or {min: 1}; counted over the whole run
  never_error: true         # health never ERROR
  ends_ok: true             # last health report is OK
  settles: true             # speed estimate within 0.05 m/s of 0 at the end
  covariance_dip: {min_drop: 0.03}  # out and back: variance falls, then grows
expected_fail: "why"        # known failure: passes while a check fails
```

After the last step the car is stopped and recorded for 1 s more. A
`/odom` jump of more than 0.1 m between samples always fails.

The car spawns at the origin facing +x. The world is a flat 100 x 100 m
plane, so keep scenarios within about 40 m of the start.

Run one: `make test-system FILTER=test_scenario_<name>`. Each writes
`scenario_<name>_trace.csv` (truth against estimate) to
`test/system/results/`.
