> Archived historical report, superseded by [VALIDATION.md](VALIDATION.md).
> Statements about the current environment below refer to the earlier session.

# First closed-loop campus drive — validation status

Work resumed on 2026-10-17/18 UTC. **The existing IDEAL_STEER_ACC recordings establish
closed-loop tracking, but do not establish a controlled terminal standstill or
safe feedback-loss response. Overall success is not yet demonstrated.**

## Current execution environment

`results/current_audit/runtime_preflight.json` records `[Errno 1] Operation not
permitted` for creation of a localhost UDP socket and inaccessible X11 `:0`.
A real rclpy probe independently emitted `TRANSPORT_UDP Error creating socket`
and `User transport failed to register` (`dds_probe.log`). Node creation alone
succeeded but does not establish working DDS communication. The runner now
fails before launch in this environment. Fresh full-route, geared-model,
feedback-fault and RViz runtime tests are **not executed/verified** here. The
geared corrective experiment below is prepared, not a measured success.
An actual bounded RViz invocation with this configuration exited 134 because Qt
could not connect to display `:0` (`rviz_attempt.log` / `rviz_attempt.json`).
Current-session RViz subscriptions and rendering could therefore not be checked.

## Pinned source and implementation

Source of truth is the local Autoware **1.9.0** checkout, not current online
documentation. The audited commits are recorded in
`results/current_audit/pinned_sources.json`; upstream working trees were clean.

- Autoware: `10718787ba6e28f038a0cb29ff99cc627b5abfd2`.
- Core: `f25f83c632c1984ec276c894c41857d4abc0dad8`.
- Universe: `6e477c645efec33f7909095eea684474e97f5e3d`.
- Simple simulator: `82d2c93a28a25f8802e8f5a21b4ec850beea9080`.

Actual control implementation: `autoware_trajectory_follower_node`, executable
`controller_node_exe`, selecting `autoware_mpc_lateral_controller` (MPC,
`kinematics_no_delay`) and `autoware_pid_longitudinal_controller` (PID).
`Controller::callbackTimerControl()` combines their lateral and longitudinal
outputs into `autoware_control_msgs/msg/Control`. Its configured period is
0.03 s; command-age timeout is 0.5 s. The command-age check is not an input
freshness safeguard: the polling subscribers can retain feedback after loss.
The recorded drop/freeze tests confirm continued commands and motion.

The optional integration uses pinned `autoware_vehicle_cmd_gate` executable
`vehicle_cmd_gate_exe`. A test-only watchdog checks original ROS timestamps for
odometry, steering, acceleration, trajectory and controller command. It withholds
the external emergency heartbeat after any input exceeds 0.3 s, and latches the
fault. The pinned gate then generates emergency braking at -0.5 m/s² after its
0.3 s heartbeat timeout. The watchdog never generates control commands.

## Pipeline and topic mappings

All control/simulator topics are isolated under `/atlas_closed_loop` in ROS domain
79. Map publication is under `/atlas_map_demo`. No physical vehicle interface is
launched. The actual map routing computes lanelets **38184 → 23892**, length
**69.779 m**, with the original map projection and no alignment correction.
The path generator, pinned path conversion, and Analytical velocity smoother
produce the live trajectory with a 1.0 m/s cap and zero terminal target speed.
`route_adapter` removes lane IDs using the pinned conversion utility; it does
not generate a substitute path or speed profile.

| Pinned controller endpoint | Remapped topic | Exact message type |
| --- | --- | --- |
| `~/input/reference_trajectory` | `/atlas_closed_loop/trajectory` | `autoware_planning_msgs/msg/Trajectory` |
| `~/input/current_odometry` | `/atlas_closed_loop/feedback/odometry` | `nav_msgs/msg/Odometry` |
| `~/input/current_steering` | `/atlas_closed_loop/feedback/steering` | `autoware_vehicle_msgs/msg/SteeringReport` |
| `~/input/current_accel` | `/atlas_closed_loop/feedback/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` |
| `~/input/current_operation_mode` | `/atlas_closed_loop/operation_mode` | `autoware_adapi_v1_msgs/msg/OperationModeState` |
| `~/output/control_cmd` | `/atlas_closed_loop/control_cmd` | `autoware_control_msgs/msg/Control` |

Simulator `input/ackermann_control_command` receives the controller output
directly, or `/atlas_closed_loop/gated/control_cmd` when `gated:=true`.
The gate is then the exclusive simulator command publisher. Simulator
`output/odometry`, `output/steering`, and `output/acceleration` feed the relay,
which preserves their values and stamps. Only the simulator's hardcoded
acceleration frame `/base_link` is renamed to the identical isolated body axes.
`OperationModeState` is AUTONOMOUS with Autoware control enabled;
`GearCommand` on `sim/input/gear_command` is DRIVE. Initialization publishes the
computed route start to `sim/input/initialpose`, once. An eight-second startup
engage delay allows recording before motion. Cleanup disengagement pauses
simulation and is never counted as a controlled stop.

Frame tree is directly `map → atlas_closed_loop_base_link`, published by the
simulator, with no extra odom transform or guessed map offset. Vehicle odometry
uses those same parent/child frames. Nodes use system ROS time (`use_sim_time`
false); simulator stamps are preserved through the feedback relay. Positive
steering is left; the bicycle model uses `yaw_rate = v*tan(steer)/wheelbase`.
Positive acceleration increases forward velocity. Wheelbase is **1.20 m** in
both controller and simulator, tread 0.80 m, steering limit 0.45 rad, wheel
radius 0.20 m. All geometry is provisional, not measured Atlas calibration.
Controller acceleration bounds are ±0.5 m/s² and jerk bounds ±0.3 m/s³.
`config_consistency.json` records matching wheelbase/steering/speed settings and
confirms that the current planning overrides match the validated trajectory demo.

## Existing recorded evidence (2026-10-12; not a new execution)

The preserved `results/drive/validation.json` originally reported success under
a loose 0.02 m/s terminal criterion. Independent analysis in
`results/drive/analysis.json` disproves strict standstill. Do not interpret that
historical `passed: true` as meeting the current success criteria.

| Measurement | Existing IDEAL_STEER_ACC drive |
| --- | ---: |
| Recorded travel | 69.594847 m |
| First recorded pose gap from start | 0.229702 m (initial motion unobserved) |
| Final goal distance | 0.045908 m |
| Route-relative max / RMS perpendicular error (recomputed) | 0.003607 / 0.000730 m |
| Maximum heading error | 0.007022 rad (0.402°) |
| Maximum actual speed | 0.999924 m/s |
| Maximum acceleration / deceleration | +0.238981 / -0.317820 m/s² |
| Final speed | 0.008641 m/s — **creeping** |
| Last five seconds displacement | 0.042689 m — **not stopped** |
| Controller command rate | 32.954 Hz |
| Controller longitudinal states | DRIVE, STOPPING, STOPPED (0, 1, 2) |
| Controller debug vs live speed RMS | 0.001603 m/s (2,689 matched samples) |
| TF matched samples | 3,983 |
| TF maximum XY / yaw residual | 0.000932 m / 2.28e-9 rad |
| TF maximum matched stamp difference | 0.009799 s |

The original drive JSON's 0.045908 m lateral maximum included endpoint distance;
the independent perpendicular calculation above corrects that interpretation.
Goal distance remains a separate endpoint metric. Live feedback topology and
controller debug speed demonstrate feedback consumption, beyond publication of
commands alone. The controller reached its STOPPED state but that state alone
is insufficient evidence of physical standstill in this model.

Ungated odometry drop/freeze experiments (`results/drop`, `results/freeze`)
failed: after three seconds the simulator still moved at 0.7894 / 0.7623 m/s,
travelling 2.1128 / 2.0388 m after injection. Drop had no new feedback; freeze
replayed one original stamp. Thus the controller alone does not safely handle
these faults.

Adding the pinned gate and heartbeat watchdog with the original ungeared model
(`results/gated_drop_original`, `results/gated_freeze_original`) applied -0.5
m/s² braking, but then drove backward to about -0.103 m/s and triggered the
harness abort. These are failures, not safe stops. Initial gate filters also
logged constraint events; logs are preserved.

The saved initial trajectory was **74.690402 m / 272 points**, not the
validated 70.279 m / 226-point static snapshot. The prior closed-loop override
retained 5 m backward instead of the validated 0.5 m. Its last point matches the
route goal within 1.3 micrometres; the extra geometry is behind the start, not an
alternate forward route. `historical_geometry_review.json` records this audit.
The current override restores **0.5 m backward**, making the planning overrides
match the validated trajectory demo. Its actual runtime trajectory dimensions
remain unverified; live smoothing/resampling can change point counts as the
vehicle moves. This is another reason not to equate the saved recording with a
new execution of the exact validated snapshot.

The current independent review, `results/current_audit/historical_drive_review.json`,
also compares the raw odometry to the initial generated trajectory: maximum
lateral error **0.003011 m**, RMS **0.000735 m**, maximum heading error
**0.007271 rad**. These are trajectory-relative; the earlier table's errors are
route-relative. All numeric drive results in this document are historical
recordings reanalysed now, not a new simulation.

## Failures and corrective actions

1. Default longitudinal stopped braking accelerates the ungeared ideal model
   backward. Setting stopped acceleration to zero avoids that reverse command,
   but preserves residual velocity. The existing baseline therefore fails strict
   standstill. A shorter STOPPED entry delay solved a previous gross overshoot
   but did not solve creep (`results/failed_stopped_duration`).
2. Pinned `SimModelIdealSteerAcc::update()` integrates `dv/dt = commanded_accel`
   without DRIVE clamping. Pinned `SimModelIdealSteerAccGeared` uses the same
   bicycle equations, then clamps velocity to zero when it would reverse in
   DRIVE. The geared variant is an explicit additional experiment, not silently
   substituted for the requested IDEAL_STEER_ACC baseline. It permits a physical
   brake-hold interpretation with stopped acceleration -0.2 m/s².
3. Feedback freshness requires the test-only watchdog plus pinned gate. The
   source controller command-age timeout alone does not enforce fresh odometry.
4. Current validation requires speed below 0.001 m/s for a five-second terminal
   hold. Fault runs observe eight seconds after injection and require the final
   three seconds below 0.001 m/s and less than 1 m of post-fault travel. Tests
   return nonzero for failure. Startup motion is delayed to capture route start.
5. Current lateral metrics project onto the initial pinned smoothed trajectory,
   separately from final goal distance, and remove coincident segments.

## Build and reproduction

**Final build verification: passed.** All **64** pinned production-dependency
packages and both isolated demo packages (`atlas_map_integration_demo`,
`atlas_closed_loop_demo`) built successfully. The resumed build reused the first
seven completed packages; final invocation reused all 64 dependencies and built
the two demos. No system packages were installed. See `build_final.log`,
`build_status.json`, and `selection.json` in `results/current_audit/`.
Dynamic dependency checks passed for **25** relevant upstream executables and
libraries, and separately for `route_adapter`. The installed launch description
passed `ros2 launch ... --show-args`; Python and shell syntax checks passed.
These checks establish build/load dependencies, not runtime tracking.
The actual run entrypoint returned **2** at the environment preflight and did
not launch the simulation (`run_attempt.json`).


The former `/tmp` caches were absent on resumption. The current complete
production dependency closure contains 64 source packages; conditions are
explicitly evaluated with `ENABLE_AGNOCAST=0` and `BUILD_TESTING=OFF`.
All required system dependencies pass the local rosdep audit. Optional Agnocast
would require `liblttng-ust-dev`; it is disabled and not needed for this test.
No system installations were made. Full closure, selection, and audit are saved
under `results/current_audit/`. Controller-specific dependencies include MPC,
PID, follower base, pure pursuit (compiled executable dependency), OSQP,
Autoware utilities/messages, and optional command gate/Tier IV messages.

Builds use a durable isolated `.closed_loop_cache/` with two compiler jobs and
sequential packages, reusing completed packages. A local `colcon_sync_io.py`
runner works around stalled asyncio subprocess pipe completion in this execution
environment; it changes no compiler arguments, upstream code, or installed
colcon files. Output is buffered per subprocess. The initial resumed build completed seven
message packages, then colcon demanded a test-only `autoware_lint_common`
environment even with tests disabled. The script now excludes source packages
outside the evaluated production closure, and the resumed build reuses those
seven packages. This is an isolated build selection change, not an upstream edit.
Editing the build script while its earlier invocation was still running caused
a shell read/parse failure after all upstream compilation finished. A syntax
check and clean re-invocation resolved it; all completed builds were reused.

```bash
cd ~/roam/atlas_autoware
bash src/atlas_closed_loop_demo/scripts/build_demo.sh
# Requested model baseline; failure is expected if residual creep persists.
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --timeout 180
# Explicit alternate pinned model, gate and feedback monitor:
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --gated --vehicle-model IDEAL_STEER_ACC_GEARED --timeout 180
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --gated --vehicle-model IDEAL_STEER_ACC_GEARED --fault drop --no-rviz --timeout 40
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --gated --vehicle-model IDEAL_STEER_ACC_GEARED --fault freeze --no-rviz --timeout 40
```

Current runs write timestamped recordings under `results/current_runs/`, including
raw odometry, controller/gate commands, relayed odometry, initial trajectory,
route, diagnostics, graph publisher/subscriber evidence and validation JSON.
A normal full route has a 180-second test bound plus bounded launch cleanup.
The normal entrypoint keeps RViz enabled; `--no-rviz` omits it for fault runs.
To launch interactively, source ROS and both `.closed_loop_cache` install
setups, set `ROS_DOMAIN_ID=79`, then run
`ros2 launch atlas_closed_loop_demo closed_loop.launch.py` (model and gate are
explicit launch arguments).

## RViz and remaining limits

`config/closed_loop.rviz` displays the original PCD, real Lanelet2 road markers,
selected route, generated trajectory, provisional Atlas cuboid, live TF axes,
actual driven path and goal/start markers in `map`. The historical drive recorded
all seven expected RViz topic subscriptions and consistent simulator TF.
**Rendered-image inspection remains unverified.** A cuboid and TF axes represent
the simulated vehicle; this is not a measured or detailed Atlas mesh.

PCD registration/localization is not part of this experiment: pose comes from
the simulator, not NDT. Dataset hashes are checked around every run. An ideal,
noise-free, low-speed short route does not validate real vehicle dynamics,
obstacles, wider route curvature, localization faults, or production safety.
Fault injection currently exercises odometry loss/replay; the watchdog also
checks other required inputs, but this does not prove all fault combinations.
No upstream, dataset, or production Atlas configuration edits; no physical
commands, commits, or pushes are part of this demo.
