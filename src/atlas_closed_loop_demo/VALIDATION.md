# ROAM Atlas closed-loop safety validation — 2026-10-20

**PASS for the measured headless simulation scenarios.** A fresh drive reaches the campus goal, stops, and holds zero speed for ten seconds. All six required input interruption cases are measured separately, including recovery without automatic rearming. Visual inspection is unverified.

## Environment and DDS diagnosis

Direct AF_INET UDP and TCP socket creation returns `[Errno 1] Operation not
permitted` before ROS starts. The process runs as UID 1000 with `NoNewPrivs: 1`
and seccomp mode 2. This is consistent with execution sandbox policy rather
than a ROS QoS setting or VM routing failure; the exact kernel rule was not
independently identified. No sudo, firewall, VM networking, system security,
middleware installation, or global networking changes were used.

The former socket-only preflight was an invalid inference: **installed Fast DDS
successfully delivered messages between separate publisher and subscriber
processes despite the socket diagnostics**. The fresh nonce-tagged probe received
117 distinct messages, and the full multi-process pipeline also communicates.
The transport mechanism responsible for successful local delivery was not
independently identified. The preflight now requires actual message delivery;
UDP socket results remain diagnostic. Unrestricted execution is therefore not
required for the working headless path. See `results/safety_validation/dds_probe/`
and `dds_preflight/`. RViz display access is separate.

## Fresh execution and terminal stopping

The real Lanelet2 route is 38184 → 23892, **69.778681 m**. The measured initial
pinned smoothed trajectory is **70.278690 m / 226 points**, capped at 1.0 m/s
with terminal target speed zero. Its first 0.5 m is rearward support behind the
initialized route start, so the vehicle drives the forward route rather than
travelling the rearward support segment. No pose reset, odometry falsification,
simulator freeze, map offset, or manually generated normal controls is used.

A fresh ungeared baseline reproduced the defect: STOPPED with zero commanded
acceleration retained **0.011043 m/s**, travelled **70.053356 m** over the bounded
run, and drifted to **0.274733 m** beyond the goal. It also lost valid planner
output near the goal. This baseline failed; prior recordings are not used as
proof of the corrected result.

The isolated correction selects the already-pinned **IDEAL_STEER_ACC_GEARED**
model in DRIVE, with PID stopped acceleration **-0.2 m/s²**. Its pinned gear
logic prevents negative acceleration from integrating through zero into reverse
motion, while deriving acceleration and pose from its model state. This is an
explicit model selection, not a change to pinned source or a hidden zeroing of
reported speed. The original IDEAL_STEER_ACC stopping defect remains reproducible. The diagnostic baseline command now retains improved terminal planning; its exact measurements can differ from the pre-fix capture.

The 0.5 m rearward planning extent is retained for the initial validated plan.
Below 10 m remaining, the demo uses the pinned parameter service to extend
`path_length.backward` to 5.0 m. This supplies sufficient spline points near the
end, where the 0.5 m configuration otherwise makes the pinned generator fail.
Only rearward support changes; live odometry, forward route and goal are
unchanged. Planning and smoothing continue producing fresh trajectories.

| Fresh final drive measurement | Result |
| --- | ---: |
| Total distance travelled | 69.784122 m |
| Final goal distance | 0.005497 m |
| Maximum lateral error | 0.006605 m |
| RMS lateral error | 0.001133 m |
| Maximum heading error | 0.008807 rad |
| Maximum speed | 0.999926 m/s |
| Maximum acceleration | 0.236163 m/s² |
| Maximum deceleration | -0.310014 m/s² |
| Final speed | 0.000000 m/s |
| Maximum hold speed | 0.000000 m/s |
| Total creep during hold | 0.000000 m |
| Terminal hold duration | 10.066 s |
| Stop settling time | 0.169 s |
| Raw controller command rate | 33.235 Hz |
| Unexpected motion after stopping | 0.000000 m |

Final drive checks: **38/38 passed**; failures: `[]`. Its live integration/configuration hashes match all six final fault runs (validator-only refinement documented below): **True**.

The earlier corrected development drive independently passed 34 checks:
69.770450 m travel, 0.008178 m goal distance, and 10.201 s at zero speed with zero
creep under the development validator. Its earlier hold accounting is not used as proof of the final hold; the final drive above repeats the route with recorded-time termination.
Stop settling is measured from first entering the near-goal region below
0.03 m/s until the continuous zero-speed hold begins. Terminal commands target
zero velocity and do not alternate positive acceleration with braking;
`positive_acceleration_commands_during_hold` is 0.
TF matched 4476 odometry samples; maximum matched
XY/yaw residuals are 0.000000 m / 0.000000 rad.
Maximum matched stamp difference is 0.002555 s.
Longitudinal states observed: [0, 1, 2]; no controller
EMERGENCY state occurred. The nominal watchdog never latched, and the gate never reported heartbeat timeout during the measured drive. Original dataset hashes match before/after each run.

## Feedback-loss safety

Faults interrupt only the selected delivered stream. Missing means no messages; stale means repeated original messages with unchanged timestamps. Independent true simulator odometry is recorded continuously. Controller commands are recorded before and after the fault relay, and every forwarded control payload/stamp must match an actual raw pinned-controller command. The pinned gate remains the exclusive simulator control publisher.

The watchdog requires one continuously fresh second before initial arming. It checks original ROS timestamp age and monotonic receipt age (0.3 s) for odometry, steering, acceleration, trajectory and commands. Simulator engagement waits for fresh armed status. After an armed fault, heartbeat authorization latches off; the pinned gate applies -0.5 m/s² after its 0.3 s heartbeat timeout. Recovery alone never clears either latch.

| Input | Interruption | Detection s | Stop distance m | Settling s | Final speed m/s | Post-stop travel m | Result |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| odometry | drop | 0.304 | 0.687 | 1.815 | 0.000 | 0.000000 | PASS |
| odometry | freeze | 0.306 | 0.674 | 1.801 | 0.000 | 0.000000 | PASS |
| steering | drop | 0.306 | 0.680 | 1.791 | 0.000 | 0.000000 | PASS |
| steering | freeze | 0.302 | 0.651 | 1.760 | 0.000 | 0.000000 | PASS |
| command | drop | 0.300 | 0.696 | 1.820 | 0.000 | 0.000000 | PASS |
| command | freeze | 0.286 | 0.680 | 1.800 | 0.000 | 0.000000 | PASS |

Each case restores real, advancing inputs eight seconds after injection, then
observes five more seconds without a valid rearm request. All final cases verify
fresh recovered inputs, a persistently latched monitor, zero measured speed and
no motion resumption. A rearm request while required inputs remain invalid is
explicitly refused. These tests inject at approximately 0.5 m/s; they do not
establish the same <1 m stopping distance for a fault at the 1.0 m/s speed cap.

For deliberate interactive rearming, valid inputs and three seconds at
standstill are required. Explicitly call
`/atlas_closed_loop/feedback_watchdog/rearm` (`std_srvs/srv/Trigger`), wait for
armed status with no stale inputs, then separately call
`/atlas_closed_loop/gate/vehicle_cmd_gate/service/clear_external_emergency_stop`
with the same service type. Both are isolated in ROS domain 79. There is no
automatic call to the pinned gate's clear service. Positive post-rearm driving
was not part of these interruption tests.

## Criteria declared before final evaluation

`results/safety_validation/thresholds.json` was written before the first fresh
run, and copied into each final recording. It is unchanged across the final
implementation runs.

| Check | Pass threshold |
| --- | --- |
| Initial trajectory | 226 points; 70.278681 ±0.01 m; goal endpoint within 1 mm |
| Travel / goal | >69 m travelled; <0.15 m goal distance |
| Lateral max / RMS; heading max | <0.3 m / <0.1 m; <0.15 rad |
| Maximum speed / acceleration magnitude | <1.1 m/s / <0.55 m/s² |
| Terminal stop | <0.001 m/s continuously for ≥10 s; hold travel <0.005 m |
| Settling near goal | ≤8 s after near-goal speed falls below 0.03 m/s |
| Raw controller frequency | 25–40 Hz |
| Fault detection / stopping | ≤0.4 s; travel <1 m; settling ≤4 s |
| Fault hold / recovery | final 3 s <0.001 m/s; no motion for 5 s after valid inputs recover |
| Automatic rearm | prohibited; invalid-input rearm must be refused |

The terminal run also checks no positive acceleration commands or nonzero
velocity targets during hold. Tracking errors are perpendicular distances to
the initial pinned smoothed trajectory; heading uses nearest segment tangent.
Acceleration is differentiated true odometry speed over advancing ROS stamps.
No data recorded after measurement completion is used in numerical checks.
The first final-repeat attempt failed the hold check: its wall timer expired
with only 9.834 s of recorded stopped samples. This failure is preserved in
`release_drive/`. The final validator now waits for a full hold interval in
both recorded receipt time and ROS odometry timestamps, rather than elapsed
loop time, and explicitly checks that nominal safety never latched. No stop
criterion was relaxed or simulated motion changed. The previous validator was
reconstructed and SHA-256 verified against the fault-suite version; the exact
validator-only diff is in `validator_hold_accounting.diff`. All live runtime
scripts, launch and configuration are identical across the final drive and six
fault cases; the fault evaluation logic was not changed.

## Components, topics, frames, and provisional parameters

Autoware tag **1.9.0**, root `10718787ba6e28f038a0cb29ff99cc627b5abfd2`;
core `f25f83c632c1984ec276c894c41857d4abc0dad8`;
universe `6e477c645efec33f7909095eea684474e97f5e3d`;
simulator `82d2c93a28a25f8802e8f5a21b4ec850beea9080`.
Actual package/executable: `autoware_trajectory_follower_node/controller_node_exe`,
MPC lateral (`kinematics_no_delay`) and PID longitudinal, combined into
`autoware_control_msgs/msg/Control` by the pinned controller. The gate is
`autoware_vehicle_cmd_gate/vehicle_cmd_gate_exe`. No upstream code is modified.

All topics below have prefix `/atlas_closed_loop`:

| Controller endpoint | Remapped topic | Type |
| --- | --- | --- |
| `~/input/reference_trajectory` | `/trajectory` | `autoware_planning_msgs/msg/Trajectory` |
| `~/input/current_odometry` | `/feedback/odometry` | `nav_msgs/msg/Odometry` |
| `~/input/current_steering` | `/feedback/steering` | `autoware_vehicle_msgs/msg/SteeringReport` |
| `~/input/current_accel` | `/feedback/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` |
| `~/input/current_operation_mode` | `/operation_mode` | `autoware_adapi_v1_msgs/msg/OperationModeState` |
| `~/output/control_cmd` | `/controller_raw/control_cmd` | `autoware_control_msgs/msg/Control` |

The test relay forwards raw commands unchanged to `/control_cmd` for fault
injection. The pinned gate publishes `/gated/control_cmd` to simulator
`input/ackermann_control_command`. Simulator outputs `/sim/output/odometry`,
`steering`, and `acceleration` supply live feedback; only the hardcoded body
acceleration frame name is renamed to the identical isolated body axes.
`/safety_status` records latch/freshness state. Raw, relayed, gated commands and
true/relayed state are all recorded. This relay generates no normal controls.

TF is directly `map → atlas_closed_loop_base_link`; odometry uses the same
parent/child, with no additional odom offset. System ROS timestamps are retained
through relay/fault injection. Positive steering turns left; positive
acceleration increases forward velocity. Gear is DRIVE and mode AUTONOMOUS.
Wheelbase **1.20 m** is shared by controller and simulator, steering limit
0.45 rad, wheel radius 0.20 m, tread 0.80 m, height 0.50 m. These are provisional
synthetic geometry, not measured Atlas calibration. Controller limits are
±0.5 m/s² and ±0.3 m/s³; planner cap 1.0 m/s. No physical interface is launched.

## Failures, corrective actions, and shutdown limits

- The earlier UDP-only preflight falsely blocked a usable middleware path;
  actual nonce-tagged publisher/subscriber delivery now decides readiness.
- The fresh baseline proved residual velocity under zero acceleration and
  invalid short terminal paths; the explicit pinned geared model plus rearward
  spline support resolves the measured stop while retaining live feedback.
- One development test latched before motion because authorization began after
  a single fresh observation; one-second stable freshness and armed-status
  engagement now gate startup. No post-arm freshness timeout was weakened.
- The original command-rate calculation used fault-interrupted deliveries;
  it now measures actual raw controller generation, and separately records
  delivered frequency. Missing delivered commands still must trigger stopping.
- Development cleanup sometimes required ROS launch's configured SIGTERM
  fallback five seconds after SIGINT, including different pinned node types.
  The demo now closes measurement before cleanup, explicitly stops Python test
  helpers, and requests supported component unload before ROS launch shutdown.
  Remaining bounded SIGTERM warnings are retained as operational warnings,
  never erased or presented as controller faults. Actual measurement-phase
  errors, unbounded shutdown, SIGKILL, other shutdown errors, or nonzero launch
  exit still fail checks. `operational_thresholds.json` defines this separation
  before the final implementation runs; the driving/safety thresholds were not
  relaxed. Cleanup occurs after the measured stop and cannot count as a stop.

Final drive gate-filter constraint events: **15** (filters remained enabled). Final drive DDS transport error lines: **22**; delivery, graph, feedback and movement are independently checked. Final drive SIGTERM escalations: **1**, cleanup 6.037 s. Fault-suite escalations total **2**; full warnings and runtime logs are preserved.

## Reproduction and evidence

All 64 dependency builds are reused. Only isolated demo scripts, configuration,
launch and manifest are updated/installed to `.closed_loop_cache/demo_install`.
No new system packages, upstream modifications, original map edits, production
Atlas configuration changes, physical commands, commits, or pushes.

```bash
cd ~/roam/atlas_autoware
bash src/atlas_closed_loop_demo/scripts/build_demo.sh
# Normal safe headless default, bounded to 125 s plus bounded cleanup:
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --timeout 125
# All six fault/recovery cases (each has a 45 s simulation limit):
python3 src/atlas_closed_loop_demo/scripts/run_safety_suite.py
# One case:
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --fault freeze --fault-input steering --timeout 45
# Explicit failing ungeared baseline:
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --ungated-baseline --vehicle-model IDEAL_STEER_ACC --timeout 115
# Installed-middleware delivery probe, independently of simulator:
source /opt/ros/humble/setup.bash
python3 src/atlas_closed_loop_demo/scripts/dds_probe.py
# Optional GUI, only from an accessible desktop session after headless tests:
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --rviz --timeout 125
```

Final evidence is under `results/safety_validation/`: `release_summary.json`,
`release_drive/`, `release_faults/`, `thresholds.json`, and
`operational_thresholds.json`. Each recording includes raw CSVs, initial
trajectory, route, graph, diagnostics, safety status, original dataset hashes,
source SHA-256s, threshold copy, and complete runtime log. `baseline/`, `fixed/`,
`faults/`, and `faults_final/` retain development results including failures;
they are not substituted for final evidence. Prior-session results and the
former environment-blocked report remain in `VALIDATION_PRIOR.md`.

## RViz and remaining limitations

After headless drive and all six safety tests passed, a single desktop-access probe failed: `xset: unable to open display :0`. No accessible graphical Ubuntu session was available to launch and inspect RViz. RViz subscriptions/rendering are not verified in this session; rendered-image inspection is **unverified**. No screenshots were attempted.

The RViz configuration contains Lanelet2 roads, original PCD, route, generated
trajectory, provisional vehicle cuboid and TF axes, actual driven path, and goal
markers. Headless TF/topic checks do not constitute rendered-image inspection.
No repeated screenshots or display/security bypass is attempted.

This establishes the listed bounded simulation scenarios, not production or
physical vehicle safety. Geometry/dynamics are provisional and ideal, with no
measurement noise or road slope. Fault tests are at about 0.5 m/s, not worst-case
1.0 m/s braking-distance certification. Acceleration/trajectory loss, compound
faults, clock discontinuities, positive post-rearm driving, obstacles, emergency
hardware, and watchdog-process termination are not separately exercised. Pose
comes from the simulator, not PCD registration/NDT. Remaining desktop access and
bounded shutdown warnings are independent of the measured stop and latch results.
