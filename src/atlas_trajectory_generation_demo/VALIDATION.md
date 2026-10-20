# ROAM Atlas — real-map trajectory generation and velocity smoothing

Executed 2026-10-12 UTC on Ubuntu 22.04 ARM64 / ROS 2 Humble.

**Result: PASS — 35/35 bounded runtime and numerical checks.** The pinned
Autoware path generator, PathToTrajectory converter, and Analytical velocity
smoother actually executed and published their outputs. Twenty consecutive
smoother publications were captured; all passed geometry, speed, acceleration,
and terminal-stop bounds. No controller or vehicle simulator was connected.

This validates a standard Autoware **spatial trajectory with a bounded v(s)
profile**, using a synthetic planning snapshot. It does not demonstrate closed-loop
tracking, real vehicle motion, calibrated real-cloud localization, or an absolute
time schedule. RViz runtime passed; rendered-image inspection remains unverified.

## 1. Route and fixtures retained

The route is the prepared prefix of the earlier validated campus route:
**38184 → 23892**, two lanelets in the main 2,170-lanelet component. The fixture
uses the actual Autoware-loaded map and executes `RoutingGraph::getRoute` with
the same pinned `autoware`/Vehicle rules. Predecessor/successor and traffic-rule
transitions are checked. The disconnected loading-dock component is excluded.

| Actual selected-route measurement | Value |
|---|---:|
| Route length | 69.778680829 m |
| Route Path points | 71 |
| Maximum sampled route curvature | 0.011042048 m⁻¹ |
| Centerline join gap | 0.000000 m |
| Route within original PCD XY rectangle | 100% |
| Frame | `map` |

Bounding-box coverage does not establish dense road-surface scan coverage or
PCD/OSM registration. The supplied projector is unchanged: LocalCartesian,
WGS84, latitude 51.08080935188234, longitude -114.1331143659806, altitude 0.

The existing synthetic fixture was retained: static map-frame pose at the route
start, speed **0.2 m/s**, zero acceleration, STOP operation mode with Autoware
control disabled, fresh timestamps, and `map → atlas_trajectory_ego` TF. Pose is
not integrated from velocity; this is a deliberately labelled planning snapshot,
not simulated or measured motion. The smoother logged that it planned from ego
velocity because autonomous control was disabled.

## 2. Actual pinned components

| Role | Executed implementation | Output |
|---|---|---|
| Map/projector/PCD loading | Existing pinned Autoware map components | Original map publications |
| Lanelet-based path generation | `autoware_path_generator`, `autoware::path_generator::PathGenerator` | `PathWithLaneId` |
| Message bridge | Pinned `autoware::motion_utils::convertToPath`, called by the Atlas fixture | `Path` |
| Path-to-trajectory conversion | `autoware_planning_topic_converter`, `autoware::planning_topic_converter::PathToTrajectory` | Raw `Trajectory` |
| Velocity smoothing | `autoware_velocity_smoother::VelocitySmootherNode`, algorithm `Analytical` | Final `Trajectory` |

The bridge copies header, bounds, poses, and velocities, dropping lane IDs only.
It is not a custom path, trajectory, or velocity-profile generator. Geometry and
profile generation are performed by the pinned components.

Source revisions checked by the build script:

- Autoware root, tag 1.9.0: `10718787ba6e28f038a0cb29ff99cc627b5abfd2`.
- Core, tag 1.9.0: `f25f83c632c1984ec276c894c41857d4abc0dad8`.

The experimental final constraints remain **1 m/s**, acceleration/deceleration
**±0.5 m/s²**, and configured jerk bounds ±0.3 m/s³. Vehicle geometry remains the
explicitly synthetic 1.20 m wheelbase / 0.45 rad steering limit, not measured Atlas
calibration. No map speed-limit attributes or original dataset coordinates changed.

## 3. Build and CMake results

The previously approved dependencies were installed by the user. The complete
system dependency audit now passes. No further system packages were installed.

The resumed build reused **34** cached packages and successfully built the seven
remaining packages in approximately **148.6 seconds**:

- `autoware_osqp_interface`
- `autoware_qp_interface`
- `autoware_trajectory`
- `autoware_route_handler`
- `autoware_planning_test_manager`
- `autoware_path_generator`
- `autoware_velocity_smoother`

All seven `colcon_build.rc` values are zero. No failed CMake caches existed at
resumption: only successful package caches were present. The missing packages
configured from fresh caches and found the newly supplied dependencies. There
were no build failures. Compiler output included tl_expected deprecation and GCC
ABI notes, which did not prevent compilation. Upstream source was not edited.

The local fixture target was reused. After strengthening the validator, a final
installation refresh found all **41** upstream packages available and built none
of them. It updated the local installed script without recompiling the fixture.

Artifacts remain isolated in `/tmp/atlas_trajectory_upstream_{build,install,log}`
and `/tmp/atlas_trajectory_demo_{build,install,log}`. Saved build evidence is
[results/build_summary.json](results/build_summary.json),
[results/resumed_build_selection.json](results/resumed_build_selection.json), and
[results/dependency_audit.log](results/dependency_audit.log).

## 4. Executed runtime and ROS publications

Final full run: `/tmp/atlas_trajectory_results/run_20261012_030859`.
The run lasted **11.434 seconds**, including a four-second hold after all inputs,
outputs, and required nodes were observed. Launch and validator exited 0. The
controller and simulator were absent. No critical planner/smoother/RViz errors
or solver-failure warnings were logged.

| Topic | Actual ROS type | First receipt since launch |
|---|---|---:|
| `/atlas_trajectory_demo/route` | `autoware_planning_msgs/msg/LaneletRoute` | 2.130 s |
| `/atlas_trajectory_demo/route_path` | `nav_msgs/msg/Path` | 2.132 s |
| `/atlas_trajectory_demo/synthetic/odometry` | `nav_msgs/msg/Odometry` | 2.166 s |
| `/atlas_trajectory_demo/path_with_lane_id` | `autoware_internal_planning_msgs/msg/PathWithLaneId` | 6.264 s |
| `/atlas_trajectory_demo/path` | `autoware_planning_msgs/msg/Path` | Bridge input to pinned converter; not a separate validator subscription |
| `/atlas_trajectory_demo/raw_trajectory` | `autoware_planning_msgs/msg/Trajectory` | 6.266 s |
| `/atlas_trajectory_demo/trajectory` | `autoware_planning_msgs/msg/Trajectory` | 6.270 s |
| `/atlas_trajectory_demo/trajectory_visualization` | `nav_msgs/msg/Path` | 6.276 s |

These independent subscriber receipt times include discovery, loading,
serialization, and processing; they are not isolated component latency benchmarks.
The original PCD and Autoware road markers also published and were checked.

The raw converter output inherited the map's traffic-rule velocity default:
maximum **13.888889 m/s**. It is explicitly an intermediate, not the validated
UGV trajectory. The actual smoother limited the final output to **0.999999 m/s**
and generated the terminal stopping profile. The raw and final publications are
separate topics, with no vehicle command consumer connected to either.

## 5. Measured final trajectory

The following geometry values refer to the final captured publication. Across
all twenty captured outputs, the same speed/acceleration extrema passed the
configured bounds. Detailed per-message records are in
[results/trajectory_samples.json](results/trajectory_samples.json).

| Requested measurement | Executed result |
|---|---:|
| Trajectory point count | **226** |
| Trajectory length | **70.278689687 m** |
| Maximum sampled curvature | **0.019987108 m⁻¹** |
| Maximum speed | **0.999999046 m/s** |
| Minimum speed | **0.000000000 m/s** |
| Initial planned speed | 0.200000003 m/s |
| Maximum published acceleration | **+0.214799955 m/s²** |
| Minimum published acceleration / peak deceleration | **−0.274508625 m/s²** |
| Maximum velocity-derived spatial acceleration | **+0.076135905 m/s²** |
| Minimum velocity-derived spatial acceleration | **−0.287491729 m/s²** |
| Terminal planned speed | **0.000000000 m/s** |
| Minimum / maximum point spacing | 0.100000 / 0.500000 m |
| Distance between first trajectory point and route start | 0.500000 m |
| Distance between final trajectory point and route goal | **0.000001276 m** |
| Maximum interior route-centerline deviation | **0.001824641 m** |

The additional **0.5 m** in trajectory length and the first-point offset are
explained by the configured `path_length.backward=0.5` margin. This is behind the
synthetic ego/start; it is not a lateral jump or a map-alignment correction.
The maximum clamped distance to the finite route polyline is consequently 0.5 m
at that backward extension. Interior route deviation is only 1.825 mm. Projected
route arc length advances without reversals, and every point is finite with a
normalized orientation. There are no duplicate/zero-length trajectory steps or
unexplained point-order discontinuities.

The curvature cap used in the test is `tan(0.45)/1.20`, approximately 0.403 m⁻¹.
The measured maximum is well below that synthetic-vehicle cap. This is a geometric
feasibility check, not measured Atlas steering calibration or tracking performance.

Acceleration is reported in two distinct ways:

1. The actual `TrajectoryPoint.acceleration_mps2` fields.
2. Independently from adjacent published velocities and XY arc length:
   `(v_next² - v_current²) / (2 Δs)`.

Both satisfy ±0.5 m/s² with the documented test tolerances (0.03 for fields,
0.05 for the sampled spatial derivative). Their extrema differ: equality of
stored acceleration and the velocity-profile derivative is not claimed. The
saved plot makes this distinction visible. Before relying on acceleration
feed-forward in a controller, investigate that consistency with the selected
controller and smoother configuration. Neither quantity is measured physical
vehicle acceleration.

## 6. Terminal stop and timestamp semantics

All twenty smoother outputs ended with **zero speed**. Every nonterminal portion
more than 1 m from the goal retained positive forward speed, so there was no
unexplained interior stop. This validates a terminal **planned** stop, not an
actual vehicle stopping experiment.

All captured trajectory headers were nonzero and advanced strictly; minimum
header interval was **0.098861091 s**. Header stamps matched the actual generated
path and raw trajectory stamps through the bridge/converter/smoother.

The actual output has `time_from_start=0` on every point. The pinned pipeline
therefore supplies a controller-oriented spatial velocity/acceleration profile,
not an absolute timed trajectory schedule. No point times were fabricated or
post-filled. Header timestamp checks passed; absolute time parameterization is
not claimed. A downstream consumer requiring explicit point times would need
an additional tested stage.

## 7. Numerical checks, failures, and skipped checks

The final run passed **35/35** checks, with `failed_checks: []`:

- Original selected route IDs, transition, frame, finite continuous geometry,
  length, and PCD XY rectangle containment.
- Actual generator/converter/smoother nodes alive and all required publications
  received; at least ten actual smoother outputs (twenty captured).
- Every captured output finite with normalized orientations, positive continuous
  spacing, bounded curvature, speed, acceleration, and zero terminal speed.
- Final output forward ordering, plausible length, route arc-length ordering,
  start/goal agreement, and interior centerline alignment.
- Reported and independently velocity-derived acceleration/deceleration bounds.
- No negative forward speeds or unexplained interior stops.
- Nonzero advancing headers and preservation of actual input stamps.
- RViz display subscriptions, no critical runtime errors, clean shutdown,
  and unchanged original dataset hashes.

Historical failures are retained, not counted as successful validations:

- Earlier dependency installation was blocked by sudo authentication. Resolved
  by the user's installation before this resumed build; the grouped audit now passes.
- Earlier route-only preflight exposed launch-argument scope leakage. The existing
  scoped `GroupAction` fix was retained; this resumed run did not repeat that failure.
- First full run, `/tmp/atlas_trajectory_results/run_20261012_030615`: the actual
  components ran and wrote a trajectory CSV, but the validator's JSON writer failed
  on a NumPy `bool_`. Its validation record did not save, and that run was not
  declared successful. The local writer now converts check values to Python bool.
- Corrected run `run_20261012_030654` passed 33 checks. The final run additionally
  applied output bounds to every captured sample and passed all 35.

Explicitly skipped/not performed:

- Rendered RViz image inspection; no screenshot attempt was made.
- Controller execution and simulator execution, as requested.
- Measured vehicle dynamics and actual terminal vehicle stopping.
- Real-PCD localization and calibrated PCD/OSM alignment.
- Absolute trajectory time parameterization and measured dynamic jerk validation.

Configured jerk limits are not presented as measured jerk results. No failed
trajectory check was suppressed, widened to force success, or removed.

## 8. RViz runtime

RViz initialized OpenGL 4.5 and remained alive during the hold. DDS subscriptions
were confirmed for the original PCD, Autoware Lanelet2 markers, chosen route,
synthetic start/goal/ego markers, and the actual generated trajectory visualization.
The fixed frame and all geometry are `map`. Synthetic ego TF remains explicitly
labelled and is derived from the route fixture, not from a guessed map correction.
All processes exited cleanly. The only map warning was the expected missing OSM
format-version metadata, allowed by the unchanged test loader configuration.

**Runtime visualization passed. Visual inspection is unverified.**
The saved [trajectory profile plot](results/trajectory_profile.png) is an
independent plot of actual ROS trajectory data, not an RViz screenshot.

## 9. Files updated and reproduction

Updated files in this test-only package:

- `scripts/run_validation.py`: complete-route alignment metrics, all-sample
  geometry/profile checks, JSON-safe boolean checks, explicit failed/skipped lists.
- `VALIDATION.md`: this current executed report, replacing the obsolete blocked status.
- `results/validation.json`, `trajectory.csv`, `trajectory_samples.json`,
  `runtime.log`, `selected_route.json`, build/audit evidence, and profile plot.
- Earlier route-only evidence remains in `results/route_only_validation.json`
  and `results/route_only_runtime.log`.

The route, fixtures, synthetic geometry, planning configuration, and launch pipeline
were retained. No upstream or production configuration changed. No additional
system installation, commit, or push occurred in this resumed task. Original
SHA-256 hashes matched before and after runtime.

See [README.md](README.md) for reproducible build/full-runtime commands. The build
script now audits successfully and can reuse all 41 upstream packages. Full
validation executes on isolated ROS domain 78 and shuts down after a bounded hold.

## 10. Remaining closed-loop blockers

The next technical stage is a separate, bounded controller/ground-truth simulator
integration experiment using this validated spatial trajectory. It was not
started here. Before that stage:

1. Keep the initial simulator pose and feedback in `map`; the earlier vehicle demo
   used `atlas_sim_odom`, which must not be silently equated to the campus map.
2. Use measured Atlas geometry or clearly document the synthetic-only vehicle
   experiment. Confirm steering/acceleration feedback, gear/engage state,
   operation-mode handling, and command routing/gating.
3. Verify the controller's consumption of this spatial trajectory, including
   the backward margin, terminal stop, and acceleration feed-forward consistency.
   The current fixture is static and has not demonstrated tracking.
4. For real-cloud localization, establish PCD/OSM registration and elevation
   conventions using independent evidence. Rectangle overlap does not establish
   NDT support or resolve the GNSS antenna-height issue.
5. Traffic-light/obstacle behavior, measured campus clearances, and disconnected
   loading-dock connectivity remain outside this trajectory-only test.

The trajectory-generation and smoothing milestone passed its stated bounded
runtime/numerical criteria. Autonomous driving, controller tracking, calibrated
localization, and physically measured dynamics remain unclaimed.
