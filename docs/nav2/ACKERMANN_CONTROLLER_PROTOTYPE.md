# Isolated Ackermann path-tracking prototype

## Status and boundaries

`atlas_nav2_demo.ackermann_path_controller` is a ROS-independent controller
prototype. It is not loaded into Nav2's controller server, does not publish ROS
commands, does not read the live costmap, and does not alter the existing RPP
or MPPI configurations. The existing adapter remains authoritative and
continues to reject infeasible curvature requests.

The bounded standalone harness is:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/atlas_nav2_demo python3 \
  src/atlas_nav2_demo/scripts/validate_ackermann_controller.py
```

It writes `summary.json` under
`~/roam/validation_evidence/ackermann/controller_prototype_20261111/` and
starts no ROS graph. The pytest module
`src/atlas_nav2_demo/test/test_ackermann_path_controller.py` covers individual
controller steps, geometry analysis, rollout, collision checks, and one
complete path-tracking detour.

## Kinematics and command policy

The prototype uses the same bicycle step as `ackermann_simulator`:

```text
curvature = tan(steering_angle) / wheelbase
yaw_rate  = signed_velocity * curvature
```

For a lookahead point `(x_local, y_local)` expressed in the vehicle body
frame, pure-pursuit curvature is `2*y_local/(x_local²+y_local²)`. This remains
signed correctly for a target behind the vehicle during reverse motion. The
requested steering is `atan(wheelbase * curvature)`.

Commands exceeding the curvature/steering limit are rejected and a bounded
deceleration command is returned. The controller does not clip an infeasible
request and claim the original path remains feasible. Direction changes are
separated into path runs; the tracker brakes at a reversing cusp before it
selects the opposite direction. Zero-speed commands cannot rotate the vehicle.
In-place heading changes and lateral segments inconsistent with the vehicle's
body direction are rejected instead of being skipped as if they were feasible.
Goal completion is reported only after the vehicle is inside pose tolerances
and has reached standstill; it first applies bounded deceleration.

All limits are configurable. Current fixture assumptions are wheelbase
1.20 m, steering limit 0.50 rad, minimum radius 2.197 m, steering-rate limit
0.60 rad/s, acceleration 0.80 m/s², deceleration 1.20 m/s², speed 0.80 m/s,
and a 0.32 m circular footprint plus 0.05 m margin. Wheelbase and steering
limit are existing simulation values; steering rate, acceleration,
deceleration, and footprint remain simulation assumptions, not measured Atlas
actuator or geometry data.

## Prediction and collision check

`rollout_trajectory` advances commands through the shared bicycle-model `step`
with steering-rate and acceleration/deceleration limits. Sign reversals must
pass through zero speed. The rollout returns each state, feasibility reason,
maximum steering/rate/acceleration/deceleration, and minimum clearance.

The simulator's footprint is a circle, so collision checking uses the swept
circle center segment between every pair of simulator states and exact segment
to axis-aligned-rectangle distance, inflated by the footprint radius and
configured safety margin. Map bounds are checked with the same inflation.
This checks between sampled states without missing a thin rectangular obstacle
between endpoints. It matches the discrete contact geometry in the simulator;
it is not a production perception/costmap collision monitor.

## Recorded Smac path geometry

The captured 37-pose path in
`validation_evidence/ackermann/diagnose_20261108T/capture_rpp2m.jsonl` has one
forward run and no reversing cusp. With the configured curvature limit
0.455252 1/m, maximum adjacent-pose curvature is 0.464910 1/m (radius 2.151 m),
slightly above the limit. An eight-edge same-direction window estimates
0.449611 1/m (radius 2.224 m), inside the limit. Discrete samples therefore do
not strictly certify the planner path, though the longer-window estimate is
consistent with the configured 2.20 m primitive radius. This is inconclusive
sampling evidence, not proof that the continuous Smac path is infeasible.
Planner settings were not changed.

## Deterministic results (2026-11-11)

Evidence: `~/roam/validation_evidence/ackermann/controller_prototype_20261111/`.
The initial harness failures are retained in
`summary_initial_harness_failures.json`; those exposed an out-of-bounds curved
fixture and a harness that repeated a one-tick stop setpoint instead of
requesting successive stop updates. They were corrected without controller
parameter changes.

The final fixed-step (20 ms) run passed all five standalone cases:

| Case | Result | Goal position error | Minimum modeled clearance | Peak steering |
|---|---|---:|---:|---:|
| Straight | Goal reached, stopped | 0.0515 m | 1.480 m | 0 rad |
| Curved, 0.9 rad heading change | Goal reached, stopped | 0.0757 m | 1.358 m | 0.4255 rad |
| Feasible quarter-circle obstacle detour | Goal reached, stopped | 0.0841 m | 0.1129 m | 0.4943 rad |
| Infeasible tight curve | Rejected, stopped | Not reached | 1.480 m | 0 rad applied |
| Blocked route | Rejected, stopped | Not reached | 0.00058 m | 0 rad |

Observed maximum steering rate was 0.600 rad/s, acceleration 0.800 m/s², and
deceleration 1.200 m/s². Successful cases had no rejected commands or modeled
collisions. The blocked test's remaining clearance is only 0.58 mm beyond the
configured footprint-plus-margin boundary, so it is a numerically clear
simulation stop with negligible modeled margin—not evidence of robust physical
braking distance. Failed initial harness evidence remains preserved.

## Test coverage and limitations

The focused tests cover straight/constant-radius/curved tracking, tight-turn
rejection, reverse travel and reverse-curve steering sign, cusp behavior,
predicted collision, full feasible detour, zero/low speed, invalid input,
in-place rotation rejection, steering-rate/acceleration limits, and collision
between rollout samples. The full package test suite also includes the existing
adapter and gate tests.

This prototype section records deterministic reference-controller evidence;
it is not the implementation now loaded by Nav2. The separate C++ Humble plugin
has since been integrated and tested through the live Nav2 controller server.
The recorded path's discrete curvature estimates remain inconclusive and do
not certify a continuous primitive. Current live results and limits are in
[`NAV2_INTEGRATION_STATUS.md`](NAV2_INTEGRATION_STATUS.md).

## Historical Nav2 integration prerequisites (superseded)

These were the prerequisites before plugin implementation:

1. Resolve or instrument continuous Smac primitive/path curvature and cusps.
2. Replay this controller against captured planner paths, measured odometry,
   and costmap geometry with synchronized timestamps.
3. Add a ROS wrapper or plugin only after message/frame/timeout contracts are
   reviewed; retain the command gate and adapter rejection.
4. Verify collision prediction against the Nav2 footprint/costmap and ensure
   the braking rollout is clear under timing and command latency.
5. Repeat successful scenarios and demonstrate adequate clearance with
   measured Atlas geometry and actuator limits before physical integration.

Humble API integration, live costmap collision checks, and repeated simulation
scenarios now exist. Continuous Smac primitive curvature certification remains
open; physical geometry and actuator characterization are also required before
hardware use.

No physical stopping-distance validation or hardware integration is claimed.
