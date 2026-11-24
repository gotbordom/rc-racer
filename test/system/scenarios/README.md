# Validation scenarios

One file per validation case in the register (SDP §3.6). CMake adds one
system test per file, `test_scenario_<file name>`, which runs it in its own
headless Gazebo on the empty course (`test_gnc_state_estimation_scenario`).

```yaml
id: VAL-005                 # register row
title: Full-throttle launch at mu 0.5
world: empty_course.world   # optional, the default; recordings use the drive's world
tire_mu: 0.5                # tyre friction (ground is 1.0; Gazebo uses the lower)
steps:                      # open-loop commands through steering_relay, 20 Hz
  - duration: 4.0           # s, sim time
    speed: 3.0              # m/s at the start of the step (negative = reverse)
    speed_end: 3.0          # optional: ramp linearly to this by the end
    steer: 0.0              # fraction of full lock, -1 (right) to +1 (left)
    expect_slip: true       # optional: a slip event must be counted in this step
    expect_slip_within: 0.5 # optional: ... and within this many s of its start
    min_consistent_fraction: 0.95   # optional: inside 3 sigma during this step
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
  max_jump: 0.05            # m an /odom sample may move beyond speed x dt (default 0.05)
  true_end_near: {x: 39.5, y: 7.7, within: 1.0}  # m, true end position (start frame)
expected_fail:              # known failure: passes while this check, and only it, fails
  reason: "why, and the open item"
  failing: "slip in step 2"   # text of the failure it expects (or a list of them)
```

After the last step the car is stopped and recorded for 1 s more.
state_estimation reports health at 10 Hz in these runs (1 Hz normally), so
slip detection is timed to about 0.1 s. The
continuity check always runs: an `/odom` sample may not move more than
`max_jump` (0.05 m) beyond what the estimated speed explains.

The car spawns at the origin facing +x. The empty course is a flat
100 x 100 m plane, so keep scripted scenarios within about 40 m of the
start.

## Recording a drive

With `make sim` running (restart it first, so the car is at the spawn
point), `make record NAME=<name> ID=VAL-xxx TITLE='...'` records what you
drive, pad or keyboard, until you press Enter. It writes
`<name>.yaml` here with the world and `tire_mu` of the running sim, the
steps (yaw rates turned back into steering fractions, unchanged stretches
merged) and template checks. Then:
1. say in the file what the case is, and set its real checks;
2. `make test-system FILTER=test_scenario_<name>` to run it;
3. `make replay NAME=<name>` to watch it drive the running sim.

A replay is open-loop: small differences in the physics add up, so the
car may not follow the recorded path exactly. Check properties, not the
path.

At shutdown every process must exit 0, except that Gazebo may be killed
(it sometimes hangs, #12); a warning is printed when it is.

Run one: `make test-system FILTER=test_scenario_<name>`. Each writes
`scenario_<name>_trace.csv` (truth against estimate) to
`test/system/results/`.
