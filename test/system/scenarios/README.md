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

At shutdown every process must exit 0, except the sim plumbing, which
sometimes dies (#12): Gazebo may be killed (it hangs) and the ROS-Gazebo
bridge may segfault. A warning is printed when either happens.

Run one: `make test-system FILTER=test_scenario_<name>`. Each writes
`scenario_<name>_trace.csv` (truth against estimate) to
`test/system/results/`.

## Watching a scenario

`make replay` drives a running `make sim` from a scenario file, so you can
watch it in Gazebo and RViz. Nothing is checked, and the car starts wherever
it is: **restart `make sim` before each replay**, with the scenario's world
and μ. Run `make se-watch` in a third terminal to see slip warnings
(health is 1 Hz here; the test uses 10 Hz). For pass/fail, run the test:
`make test-system FILTER=test_scenario_<name>`. (A one-command version
with the GUI on is follow-up #13.)

```
make sim WORLD=<world> MU=<mu>      # terminal 1, fresh each time
make se-watch                       # terminal 2
make replay NAME=<name>             # terminal 3
```

In RViz, the `/odom` arrow and covariance ellipse are the estimate;
Gazebo's car is the truth. "Follows" below means the arrow stays on the car.

| Scenario | `make sim` | What to look for | Known bad (expected-fail) |
|---|---|---|---|
| `val001_turbo_straight_stop` | `WORLD=empty_course.world MU=1.0` | Turbo straight and hard stop, both ways: no slip WARN in `se-watch`, the arrow follows the car, the speed settles to 0 at each stop | - |
| `val002_reverse_turns` | `WORLD=empty_course.world` | Reverse with turns: the arrow points the way the car faces (not the way it moves) and turns with it | - |
| `val003_full_lock_circles` | `WORLD=empty_course.world` | Full-lock circles, left then right: no slip WARN (yaw rate at the steering limit), the arrow stays on the car around each circle | - |
| `val004_full_lock_turbo` | `WORLD=empty_course.world` | Turbo at full lock: the car slides wide (understeer). WARN 2-3 s into each turn, back to OK, never stuck | VAL-018: the arrow drifts 1-2 m off the car while the ellipse stays small |
| `val018_sideways_slide_honest` | `WORLD=empty_course.world` | The same drive as VAL-004 | The same: the estimate is metres off with a small ellipse |
| `val005_launch_slip` | `WORLD=empty_course.world` | Full-throttle launch: the wheels spin up; WARN within ~0.5 s, back to OK; the arrow ends ~0.1-0.2 m ahead of the car | VAL-017: the arrow pulls ahead in the first ~0.5 s with a tiny ellipse |
| `val017_launch_slip_covariance` | `WORLD=empty_course.world` | The same launch as VAL-005 | The same as VAL-005 |
| `val006_braking_slip` | `WORLD=empty_course.world` | Hard stop from 3 m/s: the car slides on past where its wheels stop | VAL-006: no WARN, or only ~1 s late |
| `val016_out_and_back` | `WORLD=empty_course.world` | 10 m out and straight back: the ellipse grows on the way out, **shrinks** for the first ~5 m back, then grows again (#8). The arrow stays on the car | - |
| `val007_bench_crash` | (default: practice course) | Arc into the bench, push ~4 s, stop. The car hits the bench and stays; WARN once the wheels stop; back to OK, settles | VAL-019: while pinned the arrow drives on through the bench, ~4 m |
| `val019_pinned_estimate_stops` | (default: practice course) | The same drive as VAL-007 | The same as VAL-007 |

Pairs such as VAL-004/018 drive the same steps and differ only in what
their tests check, so watching one shows both.
