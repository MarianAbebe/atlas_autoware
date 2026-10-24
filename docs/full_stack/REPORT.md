# Experimental full-stack driving prototype

Executed 2026-10-22 on the existing ARM64 / ROS Humble / 4 GB VM, within a
45-minute timebox. **All requested processing connections execute, but repeatable
nominal obstacle stopping remains unproven.** The unchanged real-time checks
rejected one of two obstacle attempts. No watchdog threshold, pinned upstream
source, original map or recording was modified. No packages were installed and
no commit or push was performed.

[Build, launch, topic inspection and RViz instructions](../../src/atlas_full_stack_demo/README.md)
are in the new `atlas_full_stack_demo` package.

## Scope and integration decision

The supplied campus PCD still lacks verified registration to the vector map.
Rather than assert that missing alignment, this experiment uses a **synthetic
LiDAR world/NDT reference constructed in actual campus vector-map coordinates**.
The existing pinned map loader and `route_adapter` select campus lanelets
38184 → 23892. A fixed 16 m prefix of their loaded centerline provides the test
reference; the original PCD is not used by NDT. Terrain is flat at the starting
height, with asymmetric static roadside landmarks and a stationary on-route block
at distance 10 m. This is not real-PCD localization or a full campus mission.

The existing localization launch now exposes optional composition parameters for
its sensor input, smoother input, fixture inclusion and monitor output remaps.
Calling it normally retains the original nodes, order, parameters and topics.
The new launch calls that factory without the old synthetic fixture, then adds
existing campus map/route nodes, preprocessing, clustering, obstacle-stop planner
and perception authorization guard. No simulator, estimator or controller was
rewritten. Each process is visible in the launch log and
[the recorded process list](manual/processes.json).

The chain is:

1. Hidden pinned-simulator state generates map-consistent synthetic Livox-layout
   scans and ideal body speed/gyro/acceleration measurements.
2. Existing Livox converter → pinned distortion corrector → compact converter.
   Compact clouds branch to NDT and the pinned ground filter.
3. Pinned ground filter → pinned Euclidean clustering → measured geometry/static
   object adapter → pinned obstacle-stop planner.
4. Pinned NDT poses correct pinned EKF; EKF prediction poses feed NDT and EKF
   odometry exclusively feeds MPC, smoother, planner and gate.
5. Pinned stop planner → Analytical smoother → MPC/PID → pinned gate → simulator.
6. Unchanged localization watchdog and existing perception guard authorize the
   gate. Both retain **0.300 s** freshness and existing latch semantics.

The sensor frame is `synthetic_lidar`, co-located with virtual `base_link` by
construction. All preprocessing stages preserve acquisition stamps. FLOAT64
absolute point times are quantized together with the header to 256 ns, avoiding
loss of exact integer representation at this epoch. All points in a scan are
instantaneous: deskew executes, but no rolling-scan motion-compensation claim is
made. No future poses, refreshed stale stamps or truth odometry fallback exists.

## Object contract adaptation and causal stop path

Pinned clustering supplies UNKNOWN-labelled centroids and a colored membership
cloud; its `DetectedObject` shape/orientation/existence fields are unset. A small
adapter joins those outputs by exact acquisition header, checks membership
centroids, derives measured axis-aligned bounds, transforms through **EKF TF**,
and supplies stationary five-second predictions. It does not read truth, scene
obstacle dimensions or fixture labels. It does not re-cluster or replace the
pinned detection algorithm. Its existence=1 and constant paths are explicit
static-scene assumptions, not learned confidence, tracking or motion estimates.

For a meaningful causal test, the experiment disables only the planner's
**parallel point-cloud stop candidate branch**. The real filtered cloud stays
connected and freshness monitored. Actual parameter-service readback verifies
`obstacle_stop.obstacle_filtering.check_inside.pointcloud=false`; therefore the
planned stop must use detector-derived objects. The new configuration does not
change safety timeouts or braking limits.

Actual graph edges, exclusive publishers, acquisition-stamp lineage, conversion
point counts, input frames and observed finite geometry are checked during and
after the runs. The offline reviewer additionally checks every chain edge,
controller command bounds, actual speed, and absence of truth subscriptions at
MPC, EKF, NDT, planner, smoother, gate and object adapter. Matching scan samples
transformed with independent truth stay fixed in the world within approximately
1.2 micrometres while sensor coordinates evolve over the drive.

## Actual point contracts

A read-only capture collected all six formats for the **same acquisition**:
[contracts.json](inspection/contracts.json).

| Stage | Points | Bytes/point | Relevant fields |
|---|---:|---:|---|
| Raw | 3969 | 26 | FLOAT32 XYZ/intensity; UINT8 tag/line; FLOAT64 absolute-ns timestamp |
| Extended | 3969 | 32 | FLOAT32 XYZ; UINT8 intensity/return; UINT16 channel; FLOAT32 angles/range; UINT32 relative time |
| Deskewed | 3969 | 32 | same extended schema |
| Compact | 3969 | 16 | FLOAT32 XYZ; UINT8 intensity/return; UINT16 channel |
| Ground-filter output | 1667 | 16 | compact schema retained; removed ground is intentional |
| Cluster membership | 1667 | 16 | FLOAT32 XYZ/rgb; contiguous colors encode actual pinned membership |

All recorded frames here are `synthetic_lidar`. Ground filtering does not convert
this cloud to an assumed float-intensity layout; clustering and NDT consume its
actual XYZ fields. Full offsets, datatypes and strides are retained in the JSON.
The common-scan count checks confirm conversion does not drop points. Ground
removal and minimum cluster rules are separate intentional perception operations.

## Results: exact attempts, no selected-success reporting

The first six-case batch used unchanged online component/configuration versions.
Two additional isolated NDT-deactivation cases distinguish localization loss from
shared-sensor loss. All attempts remain under [validation/](validation/).

| Scenario | Passed / attempted | Meaning |
|---|---:|---|
| Stationary obstacle | **1/2** | one planned stop/hold; one perception freshness failure |
| Clear route | **2/2** | 16 m goal and ten-second stopped hold |
| Isolated NDT deactivation | **2/2** | LiDAR/perception continue; stale absolute pose stops driving |
| Shared-LiDAR loss | **2/2** | both localization and perception inputs disappear; bounded stop |

The successful obstacle run drove approximately 6 m and held ten seconds with
**2.104 m projected front-bumper clearance** to the known synthetic obstacle.
This includes the configured 1.5 m front offset and approximate 0.4 m obstacle
half-length; it is a synthetic geometric metric, not physical calibration.
No watchdog latch occurred in that successful run. The obstacle is visible from
startup; this does not test an object first appearing at speed.

| Run | Max NDT XY error | Max EKF XY error | Max distance to reference polyline | Final goal error / obstacle clearance |
|---|---:|---:|---:|---:|
| Obstacle 1, PASS | 0.0819 m | 0.0843 m | 0.0601 m | 2.104 m clearance |
| Clear 1, PASS | 0.0959 m | 0.0993 m | 0.0743 m | 0.1097 m goal error |
| Clear 2, PASS | 0.0944 m | 0.0975 m | 0.0745 m | 0.0934 m goal error |

Errors are post-engagement against independent hidden truth, nearest timestamps
within 25 ms. They include a visible roughly 8 cm synthetic NDT bias; this is not
truth passthrough and is not a localization-accuracy claim for real sensors.
Tracking is computed offline against line segments, avoiding the coarser online
nearest-waypoint metric. Complete max/RMS values and counts are in
[summary.json](validation/summary.json).

The two isolated NDT-loss cases latched specifically on accepted NDT at
**0.277 / 0.303 s** after the deactivation request. They stopped after approximately
**0.721 / 0.738 m**, settling in **1.820 / 1.840 s**. At the decisions, NDT source
ages were 318 / 306 ms, while EKF odometry ages were only 8 / 19 ms. Thus fresh
prediction publication did not bypass absolute-localization freshness. LiDAR,
filtered clouds, clusters and predicted objects continued after deactivation;
accepted NDT age exceeded 12 s by the measurement end. No automatic rearm or
truth-state substitution occurred.

### Preserved nominal failure

`obstacle_2` engaged but latched at evaluator-relative **7.386954 s** on
`predicted_objects`. It moved only about 0.06 m, then remained stopped with the
latch; the obstacle hold/completion test failed. Its geometric scan consistency
was good, but insufficient motion also failed the evolving-scan check. This is
**not** counted as successful obstacle stopping.

Independent records show a roughly 250 ms detector/object observation gap and a
missed raw acquisition interval nearby. A newer object was observed near the
latch, but that does not establish delivery or callback execution at the guard's
own reader. NDT remained available with a maximum observed acquisition gap near
200 ms. No TF/membership-adapter error was logged at that event. The originating
host/executor/transport condition remains unproven; this is not represented as a
fixed timing problem or resolved by increasing thresholds.

### Processing time and resources

For successful `obstacle_1`, acquisition-to-observer latency was:

| Output | Median | P95 | Maximum |
|---|---:|---:|---:|
| Accepted NDT | 34.6 ms | 36.8 ms | 51.3 ms |
| Detected clusters | 35.8 ms | 38.8 ms | 69.4 ms |
| Measured static predictions | 40.8 ms | 44.2 ms | 72.7 ms |

These include upstream processing, middleware delivery and this evaluator's
callback scheduling. They are not per-node CPU timings or a separation of
transport from executor delay. The first observed matched-object to first
observed stop-plan interval was about **0.928 s**; that is an observed reaction
interval, not a traced causal latency through internal planner callbacks.

Sampled process-group aggregate RSS was approximately **1.21–1.27 GiB** across
the first six runs, within the 4 GB VM budget. It double-counts shared pages,
excludes the evaluator and is not an exact physical-memory peak. Build jobs are
serialized. No recording was opened or replayed.

## Development failures and regressions

The early attempts are retained in the verified handoff backup under
`payload/repository/atlas_autoware/docs/full_stack/development/`:

- `obstacle_01`: the initial adapter lacked prediction paths. The planner warned
  and did not insert a stop; the simulation passed through the obstacle and
  eventually stopped at the route endpoint. Its validation **failed**.
- `obstacle_02`: paths existed, but centroid-only zero-size shapes still could not
  request a stop. A new independent clearance-abort guard ended the test before
  the obstacle. Its validation **failed**.
- `obstacle_03`: measured membership bounds completed the object contract. The
  planned stop and ten-second hold **passed**, with about 2.116 m clearance.

These are implementation-development attempts, not hidden retries of the final
configuration. The clearance abort is test cleanup and cannot count as controlled
braking. No safety threshold was weakened to obtain the later pass.

Targeted preservation checks, rather than another full 18-case suite:

| Check | Result |
|---|---|
| New geometry/header/membership/static-prediction contracts | **4/4 PASS** |
| Existing converter CTest targets | **2/2 PASS** |
| Existing standalone NDT + preprocessing-to-NDT | **2/2 PASS** |
| Existing localization default-launch nominal smoke | **PASS** |
| Manual launch + operator helper | engaged; ended stopped and unlatched |
| Read-only actual format/parameter capture | **PASS** |

The converter targets are CTest executables, not a claim that only two individual
assertions exist. No whole obstacle-stop/control suite was rerun here; the retained
latest old results remain 10/11 and 7/7, and their intermittent failures are not
assumed fixed. RViz configuration/instructions are provided but rendering was not
validated in this headless session.

Initial daemon-based inspection commands failed due the tool's local socket
restriction. Daemon-free topic and parameter queries work; one short discovery
query missed the guard node. Direct parameter-service capture verified both
0.3 s thresholds, and the subsequent query with `--spin-time 3` succeeded. All
attempts are retained in [manual/](manual/) and [inspection/](inspection/), with
the full `contracts/` export in the verified handoff backup. The new adapter's
context-shutdown handling was then
made explicit; the final inspection launch exits without its earlier shutdown
traceback. Runtime errors while ROS is active still propagate.

## Remaining blockers

1. **Nominal reliability:** the new obstacle case is 1/2. Trace the actual guard
   reader, callback/timer boundaries and sensor scheduling at a failure before
   changing online execution. Full logical-time coordination still conflicts
   with pinned controller clock contracts and monotonic safety freshness.
2. **Real map integration:** establish supplied PCD/vector registration and a
   defensible sensor/map model before claiming campus-PCD localization. The
   present world is synthetic, planar, all-visible and noise-free, not ray traced.
3. **General perception:** measured axis-aligned bounds and stationary predictions
   are not tracking, dynamic-motion estimation, calibrated detection confidence
   or support for arbitrary 3D orientation. Missing membership/TF withholds output
   and can conservatively cause a freshness stop.
4. **Qualification:** two attempts per scenario do not establish reliability,
   dynamic obstacle safety, physical performance or full-campus mission planning.

The independently runnable prototype and evidence are complete at this scope.
Repeatable full-stack success and real-campus PCD localization are not claimed.


Source and installed-library hashes, pinned revisions, unchanged original-map
hashes and driving-component version checks are in
[source_evidence.json](source_evidence.json). The only post-batch adapter change
handles invalid-context shutdown exceptions; active-context failures still raise.
Final contract inspection verified that shutdown path without a traceback.

Completed 2026-10-22T22:37:49.298894+00:00; stopped within the 45-minute timebox.
