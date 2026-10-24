# Raw LiDAR to obstacle planning (isolated simulation)

This demo executes Atlas Livox-to-extended conversion, unchanged pinned Autoware
deskew, Atlas extended-to-compact conversion and pinned Core ground filtering
before the existing pinned obstacle-stop planner, smoother, MPC/PID and simulator.
It publishes synthetic **raw** scans only. It never injects a nonground cloud or
control command. See VALIDATION.md for measured status and limitations.

## Build and run

```bash
cd ~/roam/atlas_autoware
bash src/atlas_perception_planning_demo/scripts/build_demo.sh
python3 src/atlas_perception_planning_demo/scripts/run_suite.py
# Individual bounded scenario (ahead includes both blocks, then removal/resume):
bash src/atlas_perception_planning_demo/scripts/run_test.sh --scenario ahead
# Missing / frozen original-timestamp raw scans:
bash src/atlas_perception_planning_demo/scripts/run_test.sh --scenario stale_drop
bash src/atlas_perception_planning_demo/scripts/run_test.sh --scenario stale_freeze
```

The build audits the full non-test closure and never installs system packages.
It reuses `.closed_loop_cache`, `.obstacle_cache` and `.perception_cache`. Atlas
converter and pinned deskewer libraries whose former /tmp caches disappeared
are rebuilt unchanged in the new ignored cache. The ground filter and point
types are the only newly needed pinned packages. Results remain ignored under
`results/`; each new full suite has a timestamped directory and an offline audit.
Only one suite should use ROS domain **81** at a time. No production bridge or
physical interface is launched. Workstation map/reference paths are inherited.

## Actual cloud path

| Stage | Topic below /atlas_closed_loop | Format |
| --- | --- | --- |
| Synthetic raw acquisition | lidar/raw | Livox 26-byte XYZ, FLOAT32 intensity, tag/line, absolute FLOAT64 nanoseconds |
| Atlas LivoxToExtendedComponent | lidar/extended | 32-byte XYZIRCAEDT, relative UINT32 point offsets |
| Pinned DistortionCorrectorComponent | lidar/deskewed | Same 32-byte layout and header; corrected XYZ |
| Atlas ExtendedToCompactComponent | lidar/compact | 16-byte XYZIRC |
| Pinned GroundFilterComponent | obstacles/pointcloud | Selected 16-byte nonground points; original header |
| Pinned ObstacleStopModule | obstacle_trajectory | Autoware Trajectory with inserted stop |
| Pinned final smoother | trajectory | Controller reference at 1 m/s cap |

The ground filter is required because the obstacle planner expects a nonground
cloud, not arbitrary raw scan returns. Euclidean DetectedObjects output is not a
substitute for this cloud. No standalone Euclidean object detector or real LiDAR
is run; the planner retains its own internal cloud preprocessing/clustering.
The required PredictedObjects stream is explicitly empty readiness scaffolding.

The synthetic LiDAR origin coincides with the provisional base_link axes.
All cloud stages retain `atlas_closed_loop_base_link`; true simulator TF supplies
`map -> atlas_closed_loop_base_link`. No guessed map offset or fabricated sensor
extrinsic is supplied. The fixture has a declared 1 m blind zone and a flat ground
grid; it is not a physical sensor model. Normal scans contain 2,598 ground points,
plus 315 points for each active block. Ahead has both an on-route and a 4 m
lateral block; off-route has only the lateral block. Intensity labels 10/200/150
are audit ground truth only; algorithms classify by geometry, never those labels.

Scans span 40 ms, with headers 80 ms before publication. Each raw point is
expressed at its acquisition pose using buffered true simulator state. Per-point
absolute nanoseconds are exactly representable (256 ns quantization). Live
simulator twist with original stamps supplies the unchanged 2D deskewer. This
models motion distortion and allows measuring correction against static world
geometry, rather than testing an all-zero-offset passthrough.

## Safety and recovery

The unchanged original watchdog handles odometry, steering, acceleration,
trajectory and control freshness. An added simulation-only guard checks original
stamps and receipt age at **every** perception stage, plus original authorization.
A stale or missing raw scan must not remain authorized through a downstream
republication. The guard withholds heartbeat on a latched fault; the pinned gate
provides braking. No custom stop trajectory or acceleration is generated.

Fresh obstacle removal permits normal replanning. Fresh recovery after a fault
never automatically rearms. Explicit recovery requires fresh data and three
seconds of live standstill, rearm of any latched original watchdog, rearm of
`/atlas_closed_loop/obstacle_guard/rearm`, then a separate pinned gate clear.
Service names and types match the obstacle-stopping demo, isolated on domain 81.
Positive post-fault driving after a valid reset is outside this suite.

The 1 m sampling exclusion is applied to the synthetic ground grid; the tested
blocks always remain outside that region. Close-range obstacle acquisition and
physical self-masking are not validated by this fixture.
