# Obstacle-aware campus stopping — fresh measured validation

**PASS for the bounded headless simulation scenarios executed on 2026-10-20.**
The actual pinned obstacle-stop plugin inserts a stop before the injected block;
pinned MPC/PID drives the simulator to standstill with measured clearance and
zero hold creep. Fresh obstacle removal permits normal resumption. Perception
and vehicle-feedback faults cause measured stopping and remain latched after
valid data returns. This is not physical-vehicle collision-avoidance certification.

## Fresh numerical results

All three route runs retain the real 38184 → 23892 route, 69.778681 m. Initial
smoothed trajectories are 226 points / 70.278690 m, including 0.5 m of rearward
support which the vehicle does not drive. Maximum actual speeds stay below
1 m/s. All normal final speeds and terminal hold travel are **0**.

| Route scenario | Checks passed | Travel m | Final goal distance m | Max speed m/s | Max deceleration m/s² | Lateral max / RMS m | Terminal hold s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| No obstacle | 20/20 | 69.771917 | 0.006712 | 0.999926 | -0.311086 | 0.006643 / 0.001116 | 10.060 |
| Ahead, hold, clear, resume | 28/28 | 69.763071 | 0.015558 | 0.999564 | -0.311900 | 0.006548 / 0.001032 | 10.060 |
| 4 m off route | 21/21 | 69.822417 | 0.043791 | 0.999927 | -0.311817 | 0.006692 / 0.001113 | 10.060 |

| Obstacle measurement, final ahead/clearance run | Measured result |
| --- | ---: |
| Speed at injection | 0.722623 m/s |
| Initial obstacle center distance | 14.859658 m |
| Initial footprint clearance | 12.957857 m |
| Minimum active-obstacle clearance | 1.976778 m |
| Stopped vehicle-to-obstacle footprint clearance | 1.976778 m |
| Travel from injection request to standstill | 10.982865 m |
| First received cloud to first stop-modified output | 0.551977 s |
| Fixture request to first stop-modified output | 0.558125 s |
| Maximum deceleration during obstacle window | -0.288373 m/s² |
| Maximum acceleration over complete ahead run | 0.237048 m/s² |
| Obstacle stopped hold | 5.063597 s |
| Maximum hold speed / hold travel | 0 m/s / 0 m |
| Resumption latency after fresh clearance request | 3.076157 s |
| Travel in first 15 s after clearance request | 7.179550 m |
| Final speed / terminal creep | 0 m/s / 0 m |
| Raw pinned-controller command rate | 33.335 Hz |

The block center is (61.514451, 69.139113) in map,
about 20 m along the route, with yaw 0.005162 rad. Injection
occurs after approximately 5 m of actual travel. It spans ±0.4 m longitudinally,
±0.3 m laterally and 0.1–0.4 m above route height. Clearance is the minimum
planar distance between oriented vehicle and obstacle rectangles, not a center
range. Actual received XYZ cloud geometry is checked and saved. The off-route
case offsets the same block by 4 m normal to the route. It has **zero seconds**
of false standstill outside the terminal region, and post-review finds **zero**
nonterminal stop insertions in both no-obstacle and off-route planner outputs.

## Evidence that the pinned planner caused the normal stop

The final recording pairs raw and modified trajectories by original header time.
At the inserted stop near (57.614499,
69.118979), the matching raw trajectory still requests
13.888889 m/s; the pinned plugin output inserts
zero. Raw road speeds are preserved upstream; the final smoother caps execution
at 1 m/s. No map speed tag or trajectory stop is authored by the fixture.
The 0.552 s latency is cloud-input-to-output delivery, **not** a real detector's
latency; no actual detector executes in this pipeline.

The recorded publisher graph identifies the pinned planner as the exclusive
modified-trajectory source, the final smoother as the controller reference, and
the pinned gate as the exclusive simulator control publisher. Every delivered
controller command matches an actually recorded raw pinned-controller payload
and timestamp. Simulator odometry and steering continuously update control.
Both safety monitors remain unlatched in normal runs; emergency authorization
loss cannot explain the obstacle stop. During the stopped obstacle hold,
168 raw controller commands target zero velocity; none commands positive
acceleration. Fresh clearance then produces measured motion without a safety
reset. The hold and stop are not inferred merely from a zero-speed trajectory.

## Stale perception and retained feedback-loss safety

| Required input | Interruption | Checks passed | Detection s | Stop travel m | Settling s | Max deceleration m/s² | Recovered-data stationary observation s |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| cloud | drop | 23/23 | 0.285 | 0.971 | 2.079 | -0.500081 | 5.080 |
| cloud | freeze | 23/23 | 0.274 | 0.964 | 2.079 | -0.500190 | 5.080 |
| odometry | drop | 23/23 | 0.325 | 0.991 | 2.099 | -0.500124 | 5.081 |
| odometry | freeze | 23/23 | 0.318 | 1.007 | 2.140 | -0.500102 | 5.080 |
| steering | drop | 23/23 | 0.331 | 1.019 | 2.140 | -0.500119 | 5.080 |
| steering | freeze | 23/23 | 0.334 | 1.134 | 2.240 | -0.500159 | 5.080 |
| command | drop | 23/23 | 0.314 | 0.989 | 2.100 | -0.500072 | 5.081 |
| command | freeze | 23/23 | 0.307 | 0.988 | 2.100 | -0.500098 | 5.080 |

All eight fault runs end at **0 m/s** with **0 m** recovery travel. Invalid-input
rearm requests are refused. Fresh inputs are restored eight seconds after fault
injection, followed by at least five seconds of measured zero-speed observation
without explicit rearm. Normal driving never resumes automatically. These tests
inject after 5 m of travel at about 0.7 m/s; they do not certify the same stop
travel at all speeds or under compound failures.

Perception drop/freeze deliberately interrupts an otherwise clear cloud while
moving, so an ordinary obstacle stop cannot mask the freshness safety response.
Freeze repeats the original unchanged source timestamp despite new receipts;
drop publishes no cloud. Empty PredictedObjects continues fresh. The original
feedback watchdog is unchanged; the new monitor checks both ROS stamp age and
monotonic receipt age for perception and original authorization, with a 0.3 s
limit and one healthy second before initial arming. Armed faults latch off its
heartbeat; the unchanged pinned gate supplies the configured -0.5 m/s² stopping
response after heartbeat expiry. No custom acceleration/steering command is
published by either monitor.

Normal obstacle clearance is fresh data, not a fault reset. After a fault,
recovery requires all inputs fresh and three seconds of live standstill, then
explicit rearm of any latched original feedback watchdog, explicit rearm of the
obstacle guard, and a separate pinned gate emergency clear. The tests verify
invalid reset refusal and no automatic restart; **positive driving after a valid
explicit fault reset is not tested**. See README for exact service endpoints.

## Actual pinned contracts, planning order and geometry


Local sources, not newer documentation, are the contracts:

- Root Autoware 1.9.0: `10718787ba6e28f038a0cb29ff99cc627b5abfd2`.
- Core: `f25f83c632c1984ec276c894c41857d4abc0dad8`.
- Universe: `6e477c645efec33f7909095eea684474e97f5e3d`.
- Simulator: `82d2c93a28a25f8802e8f5a21b4ec850beea9080`.

Core `planning/motion_velocity_planner/` contains the selected planner, common
library and obstacle-stop plugin, each package version **1.9.0**. The executable
is `autoware_motion_velocity_planner_node`; `plugins.xml` exports
`autoware::motion_velocity_planner::ObstacleStopModule` implementing
`PluginModuleInterface`. `launch_modules` selects that actual plugin.

`autoware_motion_velocity_planner/src/node.hpp` and `node.cpp` define the
following remaps (all right-hand names have prefix `/atlas_closed_loop`, except
the map):

| Private planner endpoint | Demo topic | Message |
| --- | --- | --- |
| `~/input/trajectory` | `/raw_trajectory` | autoware_planning_msgs/Trajectory |
| `~/input/vector_map` | `/atlas_map_demo/vector_map` | autoware_map_msgs/LaneletMapBin, transient local |
| `~/input/vehicle_odometry` | `/feedback/odometry` | nav_msgs/Odometry |
| `~/input/accel` | `/feedback/acceleration` | geometry_msgs/AccelWithCovarianceStamped |
| `~/input/dynamic_objects` | `/obstacles/predicted_objects` | autoware_perception_msgs/PredictedObjects |
| `~/input/no_ground_pointcloud` | `/obstacles/pointcloud` | sensor_msgs/PointCloud2, sensor-data QoS |
| `~/output/trajectory` | `/obstacle_trajectory` | autoware_planning_msgs/Trajectory |

`ObstacleStopModule::getRequiredSubscriptions()` requires **both** predicted
objects and nonground cloud. It does **not** consume DetectedObjects. Therefore
the fixture publishes the expected cloud directly and an explicitly
empty PredictedObjects stream to satisfy readiness. It does not invent a
DetectedObjects-to-PredictedObjects converter. The existing Euclidean detector
is not connected or claimed executed. The planner itself has point-cloud
preprocessing/clustering enabled in its pinned configuration; the nonempty-cloud stop run exercises that enabled preprocessing.

Point-cloud stop checking defaults to **false** in the pinned obstacle-stop
YAML. The isolated override enables
`obstacle_stop.obstacle_filtering.check_inside.pointcloud`. Other explicit
overrides: stop margin 2 m; terminal and minimum behavior-stop margins 1 m.
The pinned on/off hysteresis remains 0.3 s / 1 s. No upstream YAML is changed.

`process_no_ground_pointcloud()` converts incoming XYZ data to map coordinates;
it returns an explicitly empty, timestamped map cloud for empty input. Nonempty
cloud transformation can fall back to latest TF. This does not constitute a
freshness safety policy. The fixture uses `map`, avoiding a fabricated LiDAR TF.
The campus map PCD remains a map display/input, never the nonground obstacle
cloud: no ground-removal or real sensor processing is implied.

## Planning order and retained feedback

Core `autoware_core_planning/launch/autoware_core_planning.launch.xml` places
motion-velocity planning before the final velocity smoother. The executed chain
is real map → selected Lanelet2 route → pinned path generator → existing
lane-ID-removal adapter → pinned path/trajectory converter → motion velocity
planner with obstacle-stop plugin → final pinned Analytical velocity smoother →
MPC/PID controller → pinned gate → geared simulator → live feedback.

The planner's `generate_trajectory()` optionally smooths a **working copy** for
planning (`smooth_velocity_before_planning=true`), resamples it and invokes
plugins. Plugin planning results are applied to the original input trajectory;
the external final smoother then profiles that modified trajectory. It is not
valid to infer that the internal working-copy smoothing replaces final smoothing.

No controller, simulator or original safety-watchdog implementation is changed.
Provisional geometry is inherited: wheelbase 1.2 m, front/rear overhang 0.3 m,
tread 0.8 m, lateral overhang 0.1 m each, height 0.5 m. The front bumper is
**1.5 m forward of the rear-axle pose**, overall length 1.8 m, width 1.0 m.
Future clearance evaluation must account for that footprint and the obstacle's
0.8 × 0.6 m extent, not just center-to-center distance. Frames remain
`map → atlas_closed_loop_base_link`, using actual simulator TF and timestamps.


## Dependencies and successful cached builds

The previously missing `ros-humble-grid-map-core` was approved and installed by
the user: ARM64 `2.0.1-1jammy.20260307.155349`. The complete production manifest
closure contains 49 packages; system dependencies then passed. The existing
`.closed_loop_cache` overlays were reused. Exactly eight uncached pinned source
packages built successfully in `.obstacle_cache` in 4 min 13 s:

- autoware_signal_processing
- autoware_objects_of_interest_marker_interface
- autoware_object_recognition_utils
- autoware_planning_factor_interface
- autoware_behavior_velocity_planner_common
- autoware_motion_velocity_planner_common
- autoware_motion_velocity_obstacle_stop_module
- autoware_motion_velocity_planner

The isolated demo then built successfully. Compiler/dependency deprecation and
ABI notes were retained; no build failed and no upstream workaround was needed.
No additional system package was installed by this work. Build and audit logs
remain in `results/`, with caches ignored. A subsequent build reuses all eight.

## Thresholds declared before driving tests

`config/thresholds.json` predates the first obstacle run. Every final recording
contains identical values; no threshold was relaxed after seeing results.

| Check | Required limit |
| --- | --- |
| Actual speed / acceleration magnitude | ≤1.1 m/s / ≤0.55 m/s² |
| Lateral max / RMS | ≤0.3 / ≤0.1 m |
| Goal distance / terminal hold | ≤0.15 m / ≥10 s |
| Stopped speed / hold travel | <0.001 m/s / ≤0.005 m |
| Initial obstacle front clearance | 8–25 m |
| Minimum active clearance / stopped clearance | ≥0.5 m / 1–3 m |
| Fixture request to planning response | ≤1 s (also report actual first-cloud latency) |
| Obstacle hold | ≥5 s in recorded receipt and odometry ROS time |
| Normal resumption | >0.1 m/s within 8 s; ≥3 m travel within 15 s |
| Off-route false stop | ≤1 s below 0.05 m/s outside final 5 m |
| Fault detection / stop travel / settling | ≤0.4 s / ≤2.5 m / ≤5 s |
| Fresh-data recovery observation | ≥5 s; zero speed, no automatic rearm |
| Runtime bounds | 125 s normal/off-route; 160 s ahead; 45 s per fault; cleanup ≤30 s |

Acceleration is differentiated actual speed over advancing odometry ROS stamps.
Travel is the sum of successive actual XY displacements. Tracking is perpendicular
segment distance to the initial pinned smoothed trajectory. Obstacle stop travel
starts at the injection request and includes perception/planning latency and the
gentle braking approach; it is not an emergency braking-distance measurement.
Latency uses recorded receipt times, including middleware delivery. The geometric
clearance to the former block position after removal is not a collision or an
active-obstacle clearance measurement. Measurement closes before any cleanup
pause or node shutdown; cleanup cannot count as a stop.

## Fresh evidence, failures and operational observations

Final recordings are listed in `results/release_summary.json`; three complete
route runs and eight interruption runs all pass their checks with empty failure
lists. `results/runs/` retains the separate successful development drive; it is
not substituted for the final repeat. The validator gained cloud geometry and
raw/modified causal checks before the final ahead run. All final live launch,
configuration, source fixture and guard hashes match; the unchanged original
feedback/control configuration hashes are also retained. Offline review
recomputed travel from every raw CSV and checked identical declared thresholds.

The prior phase's dependency-readiness failure is resolved. No fresh motion test
failed in this phase. This does not erase unresolved limitations. DDS socket
errors remain, while a fresh two-process probe received 117 distinct messages
and actual graph, feedback and motion demonstrate delivery. Gate constraint-filter
messages and occasional MPC OSQP polishing warnings remain in full logs; filter
limits stayed enabled and numerical bounds still passed. Bounded launch SIGTERM
fallback may appear during cleanup; shutdown timing and warnings are retained
per run in the summary, separate from measured stopping.

The requested standalone perception `VALIDATION.md` was absent; its README and
test source were inspected. This experiment does **not** run that standalone
EuclideanClusterNode or real LiDAR. It uses synthetic XYZ clouds directly plus
the explicitly empty required PredictedObjects readiness stream. The pinned
motion planner's enabled internal point-cloud preprocessing/clustering executes
in the nonempty-cloud case; no DetectedObjects conversion chain is fabricated.
Original datasets and upstream working trees remain unchanged. No physical
interfaces, additional installations, commits or pushes are used.

## Exact reproduction

```bash
cd ~/roam/atlas_autoware
# Reaudit dependencies and reuse successful cached source builds:
bash src/atlas_obstacle_stopping_demo/scripts/build_demo.sh
# Complete fresh bounded suite, default isolated ROS domain 80:
python3 src/atlas_obstacle_stopping_demo/scripts/run_suite.py
# Review the exact recordings reported here without executing ROS:
python3 src/atlas_obstacle_stopping_demo/scripts/review_results.py --results-root src/atlas_obstacle_stopping_demo/results/final
# Individual cases:
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario none
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario ahead
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario off_route
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario stale_drop
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario stale_freeze
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario feedback_freeze --feedback-input steering
```

New suites default to fresh ignored `results/suite_YYYYMMDD_HHMMSS/` folders
and automatically run the offline review. The recordings evaluated here are
under `results/final/`. Individual commands default to `results/runs/`. All topics are in `/atlas_closed_loop` and `/atlas_map_demo`
on domain 80. No bridge to production is launched. Workstation-specific map and
reference paths are inherited from the previous demo; datasets are not copied.

## Remaining limitations

Ideal geared dynamics and provisional dimensions are not measured Atlas vehicle
properties. Static synthetic rectangular obstacles, noise-free simulator state,
a known route, no real localization, no real LiDAR/ground removal, no detection
accuracy claim, no object prediction and no physical braking validation. There
is no worst-case-speed, moving-obstacle, sensor miscalibration, watchdog-process
termination or compound-fault certification. Fresh-but-incorrect clear clouds
are not detected by a freshness monitor. Stale occupied-cloud behavior and valid
post-fault reset/resumed driving are not separate scenarios. RViz rendering is
**unverified**; no new graphical inspection or screenshot is claimed. These
bounded measured cases establish simulation behavior only.

## Additional per-run numerical detail

The JSON reports retain all metrics at full precision.

| Scenario / fault input | Maximum speed m/s | Maximum acceleration m/s² | Lateral max / RMS m |
| --- | ---: | ---: | ---: |
| none / odometry | 0.999926 | 0.236716 | 0.006643 / 0.001116 |
| ahead / odometry | 0.999564 | 0.237048 | 0.006548 / 0.001032 |
| off_route / odometry | 0.999927 | 0.236340 | 0.006692 / 0.001113 |
| stale_drop / odometry | 0.747841 | 0.235845 | 0.001159 / 0.000739 |
| stale_freeze / odometry | 0.752194 | 0.238093 | 0.001167 / 0.000743 |
| feedback_drop / odometry | 0.750706 | 0.236668 | 0.001171 / 0.000746 |
| feedback_freeze / odometry | 0.751621 | 0.236411 | 0.001209 / 0.000770 |
| feedback_drop / steering | 0.749902 | 0.236127 | 0.001268 / 0.000804 |
| feedback_freeze / steering | 0.809781 | 0.265363 | 0.001679 / 0.001061 |
| feedback_drop / command | 0.749523 | 0.236317 | 0.001054 / 0.000673 |
| feedback_freeze / command | 0.748979 | 0.236771 | 0.000989 / 0.000635 |

Offline review passed for all **11 recordings / 253 recorded checks**.
It also confirmed exclusive cloud and empty-object publishers, exclusive pinned
planner/smoother/controller publishers, raw travel recomputation and no
unclassified ROS ERROR records during measurement. There were **5** actual
launch SIGTERM escalations during cleanup (not ordinary `SIGINT/SIGTERM` signal
handler log messages); no SIGKILL. See `results/final/review_audit.json`.

Final syntax/configuration checks passed for seven Python files, three shell
scripts, package XML, obstacle YAML and threshold JSON. The original five-demo
fast regression/evidence checks also passed. A final dependency audit reports
49 closure entries, **zero missing source builds**, and all system dependencies
satisfied; the reuse build rebuilt no upstream package. Git status contains only
the new isolated obstacle demo directory; no tracked production files changed.
