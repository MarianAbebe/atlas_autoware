# Perception-to-planning campus simulation — 2026-10-20

**PASS for the bounded headless simulation scenarios measured here.** Actual
LiDAR conversion, pinned motion correction and ground segmentation feed the
pinned planner and cause measured obstacle stopping, normal resumption and
latched fault stopping. See the fresh results below and the retained failures.

## Fresh measured results

**All 11 selected fresh recordings pass (356 recorded checks).**
The ahead case includes both obstacles, a stopped hold, raw-scan obstacle removal,
normal resumption and complete campus route. Every route run finishes at zero
speed and zero terminal creep. The failed development attempt remains separate.

| Scenario | Checks | Travel m | Final goal gap m | Max speed m/s | Max deceleration m/s² | Lateral max / RMS m | Terminal hold s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 30/30 | 69.759566 | 0.019061 | 0.999927 | -0.332996 | 0.006658 / 0.001121 | 10.059 |
| ahead | 39/39 | 69.792500 | 0.013873 | 0.999565 | -0.311455 | 0.006573 / 0.001029 | 10.057 |
| off_route | 31/31 | 69.781955 | 0.003332 | 0.999931 | -0.311361 | 0.006706 / 0.001114 | 10.057 |

| Ahead/clearance measurement | Result |
| --- | ---: |
| Initial center range / footprint clearance | 14.862469 / 12.960667 m |
| Minimum and stopped footprint clearance | 1.965922 m |
| Travel from raw-fixture request to standstill | 10.996532 m |
| Raw occupied scan receipt to planning stop output | 0.556548 s |
| Segmented occupied cloud receipt to planning stop output | 0.547909 s |
| Maximum obstacle-window deceleration | -0.284223 m/s² |
| Maximum acceleration over full run | 0.236791 m/s² |
| Obstacle hold | 5.055162 s |
| Hold maximum speed / creep | 0 m/s / 0 m |
| Resumption after raw obstacle removal | 3.198002 s |
| Travel during first 15 s after removal | 7.083754 m |
| Controller output rate | 33.334 Hz |

Actual map anchor: (61.514451, 69.139113), yaw
0.005162 rad. The matching raw trajectory still has
13.888889 m/s at the inserted zero-speed stop
point. The final smoother caps driving to 1 m/s; no map speed tag is changed.
Both monitors remain unlatched in nominal cases. The gate is the exclusive
simulator control source, and every delivered command matches an actual pinned
controller payload/stamp. Thus the stop follows executed perception and pinned
planning, not a test-generated control command or emergency authorization loss.
The no-obstacle and off-route outputs have no nonterminal stop insertion.

### Perception measurements

| Stage | Ground-only count | Both-block scan count |
| --- | ---: | ---: |
| Raw Livox | 2598 | 3228 |
| Extended | 2598 | 3228 |
| Deskewed | 2598 | 3228 |
| Compact | 2598 | 3228 |
| Nonground output | 0 | variable, maximum 586 |

Ground leakage is **zero in every selected run**. The ahead run retains up to
291 on-route and 304
off-route points; those per-label maxima need not occur in the same scan. The
off-route-only raw scan has 2913 points. Count-preservation checks pair original
stamps through the first four stages; selection occurs only in the pinned ground
filter. Per-stage counts, labels, frame, stride, receipt time and original stamp
are saved in `perception_stages.json`.

Ahead raw-to-segmented receipt latency: mean **2.611 ms**,
maximum **12.267 ms**. This is observed message
receipt-to-receipt latency, including transport, not pure CPU profiling. Scan
acquisition/header age is additional (about 80 ms). Repeated frozen headers cannot
uniquely pair individual republish cycles; do not interpret their pairing latency
as normal processor timing. Normal advancing-stamp measurements are authoritative.

Deskew changes actual XYZ by up to **43.455 mm**.
Maximum per-scan on-route world-geometry RMS while moving is
**0.620 mm corrected**, versus
**32.443 mm uncorrected**. Each evaluated moving
scan must improve by at least 50%; values are not an all-point global RMS.
The matched TF/odometry pose and yaw residuals are zero at recorded precision in
all selected runs. Normal stage ages stay <0.3 s, with unchanged header stamps.

### Missing/stale LiDAR and original feedback cases

| Input | Fault | Checks | Detection s | Stop travel m | Settling s | Final speed m/s | Fresh-recovery hold s | Recovery creep m |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| raw LiDAR | drop | 32/32 | 0.129 | 0.850 | 1.922 | 0.000 | 5.080 | 0.000000 |
| raw LiDAR | freeze | 32/32 | 0.154 | 0.875 | 1.960 | 0.000 | 5.080 | 0.000000 |
| odometry | drop | 32/32 | 0.328 | 1.028 | 2.159 | 0.000 | 5.081 | 0.000000 |
| odometry | freeze | 32/32 | 0.338 | 1.023 | 2.160 | 0.000 | 5.076 | 0.000000 |
| steering | drop | 32/32 | 0.335 | 1.029 | 2.161 | 0.000 | 5.080 | 0.000000 |
| steering | freeze | 32/32 | 0.316 | 1.019 | 2.140 | 0.000 | 5.060 | 0.000000 |
| command | drop | 32/32 | 0.298 | 1.119 | 2.220 | 0.000 | 5.079 | 0.000000 |
| command | freeze | 32/32 | 0.303 | 0.990 | 2.097 | 0.000 | 5.081 | 0.000000 |

All invalid-input rearm requests are refused. After eight seconds, advancing raw
scans or selected vehicle feedback are restored; authorization remains latched,
and actual motion remains zero for at least five more seconds. There is no
automatic rearm. LiDAR loss starts from ground-only driving, so an ordinary
obstacle stop cannot mask freshness handling. Original cloud stamps, rather
than only new receipt times, detect frozen raw scans even when downstream
components rerun and republish. Detection from fault request can be below 0.3 s
because the most recent raw scan already has acquisition age.

## Final evidence and limitations

`results/release_summary.json` lists every selected recording and all metrics.
`results/final/review_audit.json` recomputes actual travel, checks exclusive
publishers, identical live configuration/thresholds and logs, and hashes per-run
CSV/JSON evidence. The ahead entry is a link to its original fresh recording;
its original timestamp/path are retained and are not claimed as a repeated run.
All unselected development results, including the failed zero-range case, remain.

Operational observations: gate filter messages, MPC OSQP polishing warnings and
DDS socket diagnostics persist in complete logs. Numerical bounds and actual
communication are checked independently; no security/network setting is changed.
There were 4 bounded launch SIGTERM
escalations during cleanup; no SIGKILL. Measurement closes before cleanup pause
or shutdown, so cleanup never supplies a passing standstill or hides creep.

This is an ideal geared simulator with provisional dimensions, planar geometry,
a declared near-origin ground sampling exclusion, synthetic grid returns and
known world blocks. It is not physical LiDAR, real ground/perception accuracy,
ray-traced occlusion, measured extrinsic calibration, 3D deskew, sloped/noisy
terrain, moving-object prediction or collision-avoidance certification. No
standalone Euclidean detector executes. The planner's own internal cloud
preprocessing/clustering remains enabled. Loss of the separate twist stream,
TF outages, valid post-fault reset/resumed driving, compound faults and arbitrary
fresh-but-incorrect perception are not separately tested. RViz rendered-image
inspection remains **unverified**. Original maps and all pinned upstream working
trees remain unchanged; no installations, physical commands, commits or pushes.

For an offline repeat of this recording review:

```bash
python3 src/atlas_perception_planning_demo/scripts/review_results.py --results-root src/atlas_perception_planning_demo/results/final
```

## Exact executed components and contracts

Read the milestone, obstacle-stopping and original closed-loop validation reports.
Pinned reference root: `10718787ba6e28f038a0cb29ff99cc627b5abfd2` (Autoware 1.9.0).
Core: `f25f83c632c1984ec276c894c41857d4abc0dad8`; Universe:
`6e477c645efec33f7909095eea684474e97f5e3d`; simulator:
`82d2c93a28a25f8802e8f5a21b4ec850beea9080`.

- `atlas_sensor_kit` 0.0.0: existing `LivoxToExtendedComponent` and
  `ExtendedToCompactComponent`, rebuilt unchanged because former temporary
  libraries are absent. No production converter code changed.
- `autoware_pointcloud_preprocessor` package version **0.52.0** at the pinned
  Universe commit: actual `DistortionCorrectorComponent` and supporting sources,
  compiled unchanged through the existing `atlas_distortion_corrector_test`
  0.1.0 library build. The entire preprocessor package is not claimed built.
- Core `autoware_point_types` and `autoware_ground_filter` **1.9.0**. Actual
  `autoware::ground_filter::GroundFilterComponent` is loaded from its built plugin.
- Core `autoware_motion_velocity_planner` and
  `autoware_motion_velocity_obstacle_stop_module` **1.9.0**, with actual
  `ObstacleStopModule`; existing final smoother, MPC/PID, command gate and geared
  simulator remain in the loop.

All cloud messages are `sensor_msgs/msg/PointCloud2`:

| Stage | Layout and fields | Topic below /atlas_closed_loop |
| --- | --- | --- |
| Synthetic raw Livox | 26 bytes: XYZ FLOAT32 at 0/4/8; intensity FLOAT32 at 12; tag/line UINT8 at 16/17; absolute-nanosecond FLOAT64 timestamp at 18 | lidar/raw |
| Atlas extended conversion | 32 bytes: XYZ 0/4/8; intensity/return_type UINT8 12/13; channel UINT16 14; azimuth/elevation/distance FLOAT32 16/20/24; relative UINT32 time_stamp 28 | lidar/extended |
| Pinned deskew | Same 32-byte layout, original header, corrected XYZ | lidar/deskewed |
| Atlas compact conversion | 16-byte XYZIRC: XYZ, intensity, return_type, channel; drops extended metadata | lidar/compact |
| Pinned ground filter | Selected 16-byte points with original fields/header; no ground-truth label filtering | obstacles/pointcloud |

Ground-filter source explicitly accepts the extended or compact layout. Its
`input` and `output` are sensor-data QoS. We use the compact path to exercise the
existing validated adapter. Ground segmentation is necessary: the motion planner
expects `~/input/no_ground_pointcloud` and is not a ground classifier. Standalone
Euclidean clustering publishes DetectedObjects, which cannot connect directly to
this cloud input. It is **not executed here**. No object-conversion pipeline is
fabricated. The planner's required PredictedObjects stream is explicitly empty
readiness scaffolding; the actual obstacle input comes only from ground filtering.

The launch adds no publisher of planner-input obstacle points except the pinned
ground filter. Raw → extended → deskewed → compact → segmented topics and their
exclusive publishers are checked in the recorded graph. The fixture publishes
raw scans, live-state-derived twist and empty readiness objects, never a segmented
cloud, a trajectory or a steering/acceleration command.

The selected deskew support also compiles unchanged helpers from
`managed_transform_buffer` commit `b12c9c11bccdb0ea24ac3665c91f0933ba43752f`
and autoware_utils commit `1a0ebbc8cb955e49ec28163a3a7953dc7fa4c312`.
Both working trees are clean and included in the dependency audit.

## Frames, acquisition and motion correction

Domain **81**, separate from prior domain-79/80 demos. True simulator TF is
`map → atlas_closed_loop_base_link`. Synthetic LiDAR coordinates coincide with
the provisional body axes; all cloud stages retain that frame. No physical sensor
extrinsic, guessed map correction or artificial map/odom transform is introduced.
Core ground filtering operates in those body axes, and the motion planner uses
actual TF at the retained cloud stamp to transform the result into map.

The fixture produces a flat synthetic ground grid and stationary world blocks.
Each raw point is expressed at its own acquisition pose using buffered true
simulator odometry. Scans span 40 ms; first-point headers are about 80 ms old at
publication, allowing actual motion history to be available. Absolute FLOAT64
nanoseconds are quantized to exactly representable 256 ns steps. Point offsets
are nonnegative and monotonic; converters retain header time exactly. System ROS
time is used consistently; no independent /clock or restamping conceals age.

Actual simulator twist is republished with its original timestamp and body frame
as TwistWithCovarianceStamped. The pinned deskewer uses that live twist, 2D
correction, `use_imu=false`, and `update_azimuth_and_distance=false`; the compact
adapter then drops unused extended metadata. Correction is checked as actual XYZ
change and against known static world block geometry. All-zero offsets or zero
motion are not substituted for moving scans. TF pose/yaw are compared against
actual odometry, and point counts/stamps are paired across every stage.

## Scene and provisional parameters

Ground grid: x −4…35 m, y −8…8 m, 0.5 m spacing, synthetic world z=0. A declared
1 m horizontal blind zone removes nine unrealistically close returns, leaving
**2,598 ground points**. This is a synthetic acquisition assumption, not a filter
applied after segmentation or a claim about physical Atlas coverage. The initial
zero-range attempt and failure are retained below.

A block occupies ±0.4 m longitudinally and ±0.3 m laterally, with five sampled
heights 0.2…1.0 m above route height: **315 points per block**. The ahead case
contains both the on-route block about 20 m along the route and a second block
4 m to the side. The off-route case contains only the lateral block. Normal and
cleared scans retain the full ground grid. Intensity values 10/200/150 identify
ground/on-route/off-route for evaluation only; pinned ground segmentation and
planner collision checks use geometry, not these labels.

Ground-filter defaults come from the actual pinned config; the isolated override
selects supported non-grid radial mode (`elevation_grid_mode=false`). Defaults
retain global/local slope limits 10°/13°, 1° radial bins, 0.2 m split thresholds,
virtual ground point enabled and the unchanged remaining pinned parameters.
The obstacle planner keeps the prior measured pointcloud enablement and 2 m stop
margin. Planning order remains path generator → trajectory converter → obstacle
planner → final Analytical smoother → MPC/PID → pinned gate → simulator.

Vehicle dimensions are provisional: wheelbase 1.2 m, front/rear overhang 0.3 m,
width 1.0 m, height 0.5 m; front bumper 1.5 m ahead of rear-axle pose. Clearance
uses oriented vehicle/block footprints, not center range. Real Lanelet2 route
38184 → 23892 is 69.778681 m; the initial 226-point smoothed trajectory spans
70.278690 m including 0.5 m rearward support. Speed cap remains 1 m/s.

## Safety policy and thresholds declared before evaluation

The unchanged original feedback watchdog checks odometry, steering, acceleration,
trajectory and controller commands. The new simulation-only guard additionally
checks raw, extended, deskewed, compact and segmented clouds, plus original
heartbeat/status and empty-object readiness: both original stamp age and receipt
age must be <0.3 s. One healthy second precedes initial authorization. An armed
fault latches authorization off; the pinned gate supplies −0.5 m/s² braking after
its heartbeat timeout. No custom normal or fault acceleration is published.

Fresh obstacle removal allows normal replanning and resumption. Fresh recovery
after a perception or feedback fault does not clear either latch. Explicit reset
requires valid inputs, three seconds of live standstill, rearm of any latched
original watchdog, rearm of `/atlas_closed_loop/obstacle_guard/rearm`, then a
separate pinned gate emergency clear. Invalid-input resets are tested; positive
driving after a valid fault reset is outside the suite.

`config/thresholds.json` was written before the first driving attempt and copied
into each run. Additional perception limits: ground leakage ≤1%, raw-to-segmented
receipt latency ≤0.15 s, nonzero deskew change ≥0.01 m, at least 40 retained points
from each block, corrected moving-block RMS <0.02 m and <50% of uncorrected RMS.
Normal cloud ages must remain below 0.3 s, paired stamps unchanged, and TF
position/yaw residuals below 0.03 m/rad.

Inherited motion limits: speed ≤1.1 m/s, acceleration magnitude ≤0.55 m/s²,
lateral max/RMS ≤0.3/0.1 m; initial obstacle clearance 8–25 m; active clearance
≥0.5 m and stopped clearance 1–3 m; planning response ≤1 s. Obstacle hold ≥5 s,
terminal hold ≥10 s, stopped speed <0.001 m/s and hold travel ≤0.005 m. Normal
resumption >0.1 m/s within 8 s and ≥3 m travel within 15 s. Fault detection ≤0.4 s,
stop travel ≤2.5 m, settling ≤5 s; ≥5 s at zero speed after fresh recovery, with
no automatic rearm. Normal/off-route timeout 125 s; ahead 160 s; each fault 45 s;
cleanup ≤30 s. No criterion is relaxed based on a measured outcome.

## Build, failures and regression checks

Complete production source closure: **51 packages**, initially only pinned
`autoware_point_types` and `autoware_ground_filter` uncached; system dependencies
all satisfied. Both built (36.3 s total) without upstream edits. Existing Atlas
converter and selected pinned deskewer libraries rebuilt unchanged (100 s), with
successful caches subsequently reused. No system packages were installed.

Initial demo installation failed because raw_source.py had not yet been written;
completing the isolated source and rerunning the cached build corrected it.
Plain colcon testing stalled before executing tests in this sandbox; it was
interrupted and rerun with the already-existing local blocking-I/O colcon wrapper.
No installed colcon or OS settings were changed. All **17 distinct GTest cases**
passed; colcon counts **27 entries**, zero errors/failures/skips.

`results/runs/run_20261020_221827_ahead_odometry` is an actual failed driving
attempt: ground segmentation retained one near-zero-range ground return, the
planner inserted a stop, and actual travel stayed zero. The harness aborted for
no motion; this is not counted as a successful obstacle stop. The raw fixture's
1 m blind zone removes unobservable near-origin acquisition points. No segmented
output, planner result or safety condition is manually edited. No claim is made
that the pinned ground filter robustly handles zero-range returns.

The fresh corrected ahead recording is
`results/runs/run_20261020_221951_ahead_odometry`. The final evidence directory
links to that unchanged recording rather than representing it as a new execution.
Other final cases are separate fresh runs using the same live implementation and
criteria. Complete raw recordings, stage counts, geometry checks, graph, source
hashes, thresholds, TF metrics and logs remain local under ignored results/.

## Reproduction

```bash
cd ~/roam/atlas_autoware
bash src/atlas_perception_planning_demo/scripts/build_demo.sh
# Complete new suite, fresh timestamped result root, isolated domain 81:
python3 src/atlas_perception_planning_demo/scripts/run_suite.py
# Individual runs:
bash src/atlas_perception_planning_demo/scripts/run_test.sh --scenario none
bash src/atlas_perception_planning_demo/scripts/run_test.sh --scenario ahead
bash src/atlas_perception_planning_demo/scripts/run_test.sh --scenario off_route
bash src/atlas_perception_planning_demo/scripts/run_test.sh --scenario stale_drop
bash src/atlas_perception_planning_demo/scripts/run_test.sh --scenario stale_freeze
bash src/atlas_perception_planning_demo/scripts/run_test.sh --scenario feedback_freeze --feedback-input steering
```

Use the README for cloud formats and recovery policy. Cached build and original
map paths remain workstation-specific; no maps are copied or altered. No physical
vehicle interfaces, upstream changes, commits or pushes are performed. Synthetic
raw scans are not real sensor input; standalone Euclidean detection is not claimed.

The 1 m sampling exclusion is applied to the synthetic ground grid; the tested
blocks always remain outside that region. Close-range obstacle acquisition and
physical self-masking are not validated by this fixture.

Final checks passed: seven Python files, two shell scripts, package XML, ground
YAML and threshold JSON; installed live scripts/launch match source exactly.
The final dependency audit records 51 source closure entries, no missing source
builds and all system dependencies satisfied. Original map hashes match every
recording. All six audited reference working trees are clean. The old milestone
and obstacle-demo offline regression/evidence reviews also still pass. The
pre-blind-zone fixture was recovered and verified against the failed run’s
SHA-256, archived as `results/raw_source_before_blind_zone.py`.
