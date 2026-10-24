# Recorded LiDAR perception integration

2026-10-22, ROS 2 Humble / ARM64, pinned Autoware 1.9.0.
**Five real recorded scans passed conversion, native-sensor-frame RANSAC and
Euclidean clustering, with independently checked outputs.** Clustering also
passed on the same converted scans with the plane filter bypassed. Synthetic
perception passed. Obstacle-stop regression status is recorded separately below;
a clean no-regression claim is not established by the perception results.

## Executed path and interpretation

The new test-only `real_lidar_perception_probe` loads actual registered factories
from the existing Atlas converter library, the previous milestone's unchanged
pinned RANSAC library, and pinned Core `EuclideanClusterNode`. No replacement
preprocessing or clustering algorithm is implemented. An external Python driver
selects five indexed one-second windows at 0%, 25%, 50%, 75% and near the end of
`engg_loading_dock_01.db3`, opened read-only/query-only. It reads one actual
`/velodyne_points` CDR per window, one window at a time: **3,211,265 serialized
bytes total**, no camera data and no complete replay. Each probe subprocess has
a 45-second outer deadline, and each ROS readiness/output wait a 10-second bound.

The probe forwards the exact stage outputs over ROS topics in sequence:

```text
recorded organized 22-byte cloud
  -> VelodyneToExtendedComponent (explicit header-relative seconds contract)
  -> ExtendedToCompactComponent (16-byte XYZIRC)
  -> RANSACGroundFilterComponent (same-frame 16-byte XYZIRC subset)
  -> EuclideanClusterNode -> centroid DetectedObjects + XYZRGB cluster cloud
```

A second clustering pass bypasses RANSAC and consumes the unfiltered compact
output, with the same settings. No deskew, motion samples, mounting geometry,
static TF or ground plane are supplied. The original frame
`lidar_vlp16_points_link` and sensor timestamps are preserved everywhere.
All topics are confined to `/atlas_real_perception` on domain 180. Recorded
outputs never feed the moving closed-loop simulator (domain 80).

RANSAC uses its pinned parameter YAML unchanged except base/input/output frame
names are the actual sensor frame. The managed transform buffer's same-frame
path does not need TF. The filter fits a plane to actual data and applies its
configured sensor-z inclination criterion. This is **conditional sensor-frame
plane-filter execution**, not verified physical ground segmentation. No claim
that LiDAR z equals vehicle/gravity up is made.

The RANSAC output contract was checked: unorganized, dense, little-endian,
16-byte XYZIRC with the six original fields and exact retained bytes. This is
accepted by the cluster node, which converts named XYZ fields to PCL PointXYZ;
it does not require intensity/ring for clustering. The pinned clustering YAML
specifies **use_height=false, tolerance=0.7 m, min=10, max=1000 points**.
Connectivity is therefore in the sensor's XY coordinates; z remains in cluster
output. Extra crop-box settings in that YAML do not instantiate a crop filter;
no ROI/height cropping is applied in this harness. Settings were not fitted to
reference labels or tuned between samples.

## Results

Final evidence: [perception_run_02/results.json](perception_run_02/results.json),
including per-cluster point count, centroid, XYZ min/max/extents, timing, RSS,
input CDR hashes, component/parameter hashes, and clean upstream revisions.
The parameter YAML files are copied alongside it; per-scan logs include the
middleware diagnostics. `perception_run_01` is the earlier successful execution
before provenance fields were added; final measurements below use run 02.

| Header (seconds.nanoseconds) | Converted finite points | RANSAC output | Filtered clusters | Bypass clusters |
|---|---:|---:|---:|---:|
| 1752868633.017206000 | 17,297 | 13,739 | 74 | 67 |
| 1752868757.175772000 | 19,325 | 17,603 | 47 | 44 |
| 1752868881.334336000 | 18,681 | 16,105 | 77 | 70 |
| 1752869005.392044000 | 16,173 | 13,905 | 103 | 93 |
| 1752869128.542008000 | 17,019 | 13,844 | 98 | 86 |

The driver reuses the prior independent real-converter validator: all finite
source XYZ points survive bit-for-bit after explicit stable time sorting, rings
and timestamps agree point-for-point, discarded nonfinite XYZ is counted, and
compact bytes are exact. Every filtered point is an ordered, byte-identical
subset of that compact input. All headers and layouts are asserted.

For clustering, the unchanged upstream debug output concatenates each cluster
using a cycling six-color palette, with distinct adjacent colors. Contiguous
color runs therefore expose individual cluster sizes without reimplementing
clustering. The driver requires one run per detected object, sizes 10–1000,
finite coordinates, UNKNOWN classification, and centroids agreeing with the
actual cluster mean within 1e-5 m. Multiset checks account for duplicate XYZ:
no cluster point may be invented or used more often than it occurs in its input.
Every scan produced nonempty detections on both paths. Some input points are
unclustered because connected groups outside the configured size limits are
excluded; this is reported explicitly, not treated as data loss in conversion.

Filtered cluster sizes observed were 10–909 points; bypass sizes were 10–973.
DetectedObjects from this pinned node supply centroids and UNKNOWN labels;
shape, orientation and motion remain defaults. Reported bounding extents are
diagnostics computed from debug cluster points, not object dimensions produced
by the detector. These are unlabeled point groups, not confirmed physical
obstacles, and no precision/recall or tracking accuracy is claimed.

## Timing and memory

Run 02, five cold probe processes, one scan each:

| Stage | Minimum | Median | Maximum |
|---|---:|---:|---:|
| Velodyne conversion | 3.84 ms | 4.12 ms | 5.53 ms |
| Compact conversion | 1.54 ms | 1.70 ms | 2.13 ms |
| RANSAC sensor-z plane filter | 47.61 ms | 63.10 ms | 69.90 ms |
| Clustering filtered input | 102.50 ms | 201.71 ms | 444.01 ms |
| Clustering bypass input | 134.61 ms | 228.16 ms | 467.13 ms |

Times use `steady_clock`, measured from publication to output-subscription
receipt, including middleware copies and single-threaded executor scheduling.
For clustering the measurement ends after both objects and debug cloud arrive.
The upstream wall-now-minus-historical-header diagnostic is not used.
Discovery, library loading, CDR disk export and Python validation are outside
these stage measurements. These are observed stage latencies, not isolated CPU
benchmarks or a 10 Hz real-time performance guarantee. In particular, clustering
exceeds a 100 ms scan period on this VM. Five samples do not characterize tails.

Linux `getrusage(RUSAGE_SELF).ru_maxrss` measured a maximum probe peak of
**67,100 KiB (65.5 MiB)**. The Python driver peak was 66,500 KiB (64.9 MiB).
Their sum, about 130.5 MiB, is a conservative sum of per-process maxima, not a
simultaneously sampled aggregate. Current RSS before work and after every stage
is included in JSON; those snapshots are cumulative process memory, not isolated
stage allocation peaks. Build and simulator-regression resources are excluded.

The same-process ROS component path disables rclcpp intra-process shortcuts.
The sandbox emits UDP/socket/getifaddrs errors, while local middleware delivery
completes and every output is observed. Cross-host networking, separate-process
real-time throughput, `/clock` and native bag playback remain unvalidated.

## Reproduction

First follow the conversion milestone's [dependency/build commands](CONVERSION.md)
if `/tmp/atlas_velodyne_build` is absent, enabling `ATLAS_REAL_GROUND_SMOKE=ON`.
All needed system dependencies already existed on this VM. No script below
installs packages. Use single-job builds; run simulator regressions after builds
and perception measurements finish to avoid competing for VM CPU.

```bash
cd ~/roam/atlas_autoware
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
cmake --install /tmp/atlas_velodyne_build/atlas_distortion_corrector_test
export ENABLE_AGNOCAST=0
cmake -S ~/roam/references/autoware/src/core/autoware_core/perception/autoware_euclidean_cluster_object_detector \
  -B /tmp/atlas_real_perception_upstream_build -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/tmp/atlas_real_perception_upstream_install
cmake --build /tmp/atlas_real_perception_upstream_build -j1
cmake --install /tmp/atlas_real_perception_upstream_build
export AMENT_PREFIX_PATH=/tmp/atlas_velodyne_install/atlas_sensor_kit:/tmp/atlas_velodyne_install/atlas_distortion_corrector_test:/tmp/atlas_real_perception_upstream_install:${AMENT_PREFIX_PATH}
export CMAKE_PREFIX_PATH=/tmp/atlas_velodyne_install/atlas_sensor_kit:/tmp/atlas_velodyne_install/atlas_distortion_corrector_test:/tmp/atlas_real_perception_upstream_install:${CMAKE_PREFIX_PATH}
export LD_LIBRARY_PATH=/tmp/atlas_velodyne_install/atlas_sensor_kit/lib:/tmp/atlas_velodyne_install/atlas_distortion_corrector_test/lib:/tmp/atlas_real_perception_upstream_install/lib:${LD_LIBRARY_PATH}
cmake -S src/atlas_lidar_perception_test -B /tmp/atlas_real_perception_build \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/tmp/atlas_real_perception_install \
  -DATLAS_RANSAC_LIBRARY=/tmp/atlas_velodyne_build/atlas_distortion_corrector_test/libpinned_ransac_ground.so
cmake --build /tmp/atlas_real_perception_build -j1
ctest --test-dir /tmp/atlas_real_perception_build --output-on-failure
/usr/bin/python3 scripts/validate_real_lidar_perception.py \
  /home/marian/Downloads/engg_loading_dock_01.db3 \
  --probe /tmp/atlas_real_perception_build/real_lidar_perception_probe \
  --output-dir /tmp/atlas_real_perception_results
# Separately, existing simulator-only regressions (fresh directory):
python3 src/atlas_obstacle_stopping_demo/scripts/run_suite.py \
  --results-root /tmp/atlas_obstacle_perception_regressions
```

The five-scan test is explicitly opt-in; ordinary CTest does not require the
11 GB recording. The driver exits nonzero on missing samples, failed processes,
timeouts, malformed/changed points, invalid sizes, header mismatches or
nonfinite results. It requires a fresh output directory. Temporary CDR payloads
are removed after each scan; only JSON and logs persist. Probe/library/
parameter hashes and clean upstream source revisions are recorded for review.


## Regression results and blocker

The unchanged `synthetic_lidar_obstacle_detection` CTest passed, including its
three synthetic scans, actual pinned deskew and clustering, expected centroids
and topic/layout checks. See [CTest evidence](perception_synthetic_ctest.txt).
It used the new converter-library install, retaining the original Livox path.
The prior milestone's 23-test converter/deskew report remains separate; those
23 tests were not repeated in this milestone.

All eleven existing obstacle-stop scenarios were executed with their original
thresholds. After a separately recorded fresh `none` run, the existing suite was
run with `--skip-none` and the same results root. Outcome: **8 passed, 3 failed**.
The suite returned nonzero; a successful full-suite evidence review is therefore
not claimed. [Summary and first latch records](perception_regressions/summary.json)
and individual validation/source-hash reports are archived under
`perception_regressions/`. Full runtime/trajectory/feedback recordings remain in
`/tmp/atlas_real_perception_obstacle_final` on this VM.

| Scenario | Result |
|---|---|
| stale_drop, stale_freeze | PASS |
| feedback_drop / feedback_freeze: odometry | PASS |
| feedback_drop / feedback_freeze: steering | PASS |
| feedback_drop / feedback_freeze: command | PASS |
| none (nominal route) | FAIL |
| ahead obstacle | FAIL |
| off_route obstacle | FAIL |

The failed nominal cases latched the existing feedback watchdog unexpectedly:

- `none`: first base latch at 40.50 s, reporting stale odometry, steering,
  acceleration, trajectory and command. Failed checks include controller-command
  matching, acceleration limit, route completion, nominal authorization and
  terminal hold.
- `ahead`: first base latch at 20.62 s, reporting stale steering/acceleration;
  the obstacle was never injected. Motion, command matching, route completion,
  authorization, terminal hold and injection checks failed.
- `off_route`: first base latch at 11.64 s, reporting stale odometry, steering,
  acceleration and command. Motion, route completion, authorization, terminal
  hold and the off-route false-stop check failed.

An earlier nominal run during compilation also failed (trajectory freshness
latch at about 13 s); it is saved separately as
`initial_none_during_build.json`. The later unloaded retry still failed, so
concurrent compilation alone is not established as the cause. UDP/socket
restrictions were logged, but substantial topic traffic did arrive; that does
not establish socket restrictions as the cause either. The cause of intermittent
feedback freshness violations remains unresolved. No timeout, safety threshold,
watchdog behavior or simulator code was changed to force a pass.

For all eleven final cases, recorded obstacle-demo source hashes matched current
source, thresholds matched the original configuration, and reports explicitly
identify synthetic perception with `real_lidar_executed=false`. The real-data
probe is not loaded into those launches. These are actual failed regression
checks, not evidence of an algorithm change in this isolated real-data harness.
**A clean no-regression certification is blocked until the nominal simulator
freshness failures are diagnosed and the full suite passes.**

Next work: investigate controller/feedback delivery and scheduling against the
existing 0.3 s freshness bound using the saved raw recordings, without relaxing
that bound. Separately, calibrated gravity/mounting transforms, synchronized
motion and reference labels are still required for physical ground/obstacle
claims or motion-compensated replay. The measured clustering latency also needs
work before any sustained ~10 Hz deployment claim.

No bag or pinned upstream source was modified, no system packages installed,
and no existing demo implementation or safety configuration changed. New work
is confined to a test-only perception executable, its opt-in Python validator,
CMake test-target wiring and documentation/evidence. No commits or pushes were
made; pre-existing uncommitted work was preserved.
