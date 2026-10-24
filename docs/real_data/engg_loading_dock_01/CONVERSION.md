# Organized Velodyne conversion and bounded real-data validation

2026-10-22, Ubuntu ARM64 / ROS 2 Humble, pinned Autoware 1.9.0.
**Working real-cloud conversion, original synthetic regressions preserved,
pinned deskew input accepted, and native sensor-frame RANSAC smoke passed.**
Motion compensation, calibrated ground labels and NDT localization are unvalidated.
The earlier REPORT.md and negative contract_smoke.txt remain historical audit evidence.

## Implementation

A separate `velodyne_relative_seconds_to_extended` function and registered
`atlas_sensor_kit::VelodyneToExtendedComponent` support the recorded organized
22-byte PointCloud2 format. The existing Livox and compact functions retain
their contracts and code paths. Nothing auto-detects formats or silently reroutes
existing demos. See [sensor-kit contracts](../../../src/atlas_sensor_kit/README.md)
for complete format/error policies.

The required fields are found by name and validated for datatype, count and
nonoverlapping offsets inside `point_step`. The parser visits all row/column
positions using `row_step` and `point_step`, supports row/point padding and both
message byte orders, skips/counts nonfinite XYZ, and preserves all retained XYZ
bits and the original header. Ring becomes UINT16 channel with no narrowing.
The component logs each successful cloud's retained/discarded counts and whether
retained point order changed. Output density is true because retained geometry
and generated polar values are finite. Invalid layouts or retained timestamps
reject the whole cloud transactionally.

**Time is an explicit input contract:** callers select header-relative FLOAT32
seconds via the function name or component's `time_is_relative_seconds:=true`.
The component refuses startup otherwise. This interpretation agrees with the
sampled values; decoder provenance/hardware acquisition timing is still not
established. Retained points are stable-sorted by original time: the pinned
corrector advances its motion iterators and integrates consecutive timestamps,
so the original unordered storage cannot be used directly. Equal-time order is
preserved. Seconds become relative UINT32 nanoseconds rounded to nearest, with
at most 0.5 ns added quantization. Existing source FLOAT32 precision is retained
to that interface's limit; the original float field and organized indices cannot
be carried in fixed XYZIRCAEDT. The pinned header-nanoseconds-plus-offset UINT32
sum is guarded against overflow. No restamping or coordinate transform occurs.

Intensity is clamped/rounded to UINT8 (nonfinite -> zero), unknown return type is
INVALID/0, and polar values use the existing Cartesian mathematical convention.
Compact conversion preserves the first 16 bytes, including ring/channel, and
necessarily removes per-point timing. Use extended data for any future deskew.

## Actual recorded samples

Only three clouds were read from the read-only original SQLite bag: indexed
one-second windows at start, midpoint and near end. No camera payload was
loaded. Each original cloud contains 29,184 slots (1824 rows x 16 columns),
642,048 data bytes. Total raw cloud payload read is 1,926,144 bytes; only one
window's clouds and temporary CDR outputs are held at a time.

| Header stamp (sec.nanosec) | Finite/output | Discarded XYZ | Retained time range (ns) | RANSAC output | Plane-filter removals |
|---|---:|---:|---|---:|---:|
| 1752868633.017206000 | 17,297 | 11,887 | 2,304–100,838,371 | 13,739 | 3,558 |
| 1752868881.334336000 | 18,681 | 10,503 | 4,608–100,839,369 | 16,105 | 2,576 |
| 1752869128.542008000 | 17,019 | 12,165 | 2,304–100,832,455 | 13,844 | 3,175 |

For each scan the independent Python/NumPy validator reads the original fields,
selects finite XYZ and stable-sorts expected points. It asserts exact counts,
bitwise equality of every retained XYZ, exact rings 0–15, expected intensity
quantization and timestamps for every point, finite polar values/distance,
unchanged frame `lidar_vlp16_points_link` and header, and exact compact bytes.
Every finite source point is accounted for; only explicitly counted nonfinite
XYZ slots are discarded. All three retained sets required reordering.
Maximum observed additional time rounding error was 0.499984 ns.

The probes load actual registered converter factories, publish the original
recorded message through ROS topics, and observe extended and compact outputs.
Intra-process communication is explicitly disabled. This is a single-process
middleware test, **not cross-process/network playback**. Socket/getifaddrs
restrictions emitted UDP transport errors; local topic delivery nevertheless
completed and every output was checked. A separate-process attempt failed under
these restrictions and is not claimed as validated. No `/clock` or bag-player
execution was involved; preservation of sensor headers is checked directly.

## Pinned downstream boundaries

For every actual converted extended cloud, `real_cloud_deskew_boundary` calls
the unchanged pinned algorithm's `is_pointcloud_valid`, which accepts its layout.
It then calls `undistort_pointcloud` with an actually empty twist queue. Upstream
logs `Twist queue is empty.` and returns an entirely unchanged message, as
asserted by the probe. No motion samples or extrinsics are fabricated. This is
**layout/absence-of-motion behavior**, not corrected geometry. The full node
callback is not invoked because it attempts transform setup before this check;
the audited upstream missing-transform path uses uninitialized storage. Safe
algorithm-boundary testing avoids that path without changing upstream code.

The unchanged pinned RANSAC Ground Filter factory is loaded independently, fed
the real compact output over ROS topics and run with all thresholds from the
pinned `ransac_ground_filter.param.yaml`. Only base/input/output frame names are
set to the **original LiDAR frame**. The managed buffer's same-frame path requires
no TF. This does not assert an identity LiDAR-to-vehicle extrinsic or pretend
that sensor z is calibrated vehicle/gravity up. RANSAC fits a plane from actual
points; its configured sensor-z inclination criterion is an algorithm setting.
All three clouds produced nonempty, smaller outputs. Every output point was
verified as an ordered, byte-identical subset of the compact input, including
ring/intensity, with unchanged frame and timestamp. Observed counts above are
run-specific RANSAC results, not fixed ground-label expectations.

This validates operation of a native-sensor-coordinate plane filter, **not which
points are physical ground**, ground recall/precision, motion-compensated
segmentation or vehicle-frame Scan/Ray Ground Filter integration. Those require
calibrated orientation/geometry and an independent labeling reference. No
vehicle dimensions, sensor height or ground plane were invented.

## Automated regressions

[conversion_regressions.json](conversion_regressions.json): 23 distinct GTests,
zero failures/errors, in 11 CTest processes:

- Existing converter tests: 8; existing Livox ROS component chain: 1.
- New Velodyne tests: 5, plus explicit component time-contract test: 1.
- Existing pinned deskewer scenarios: 4; existing full preprocessing chain: 4.

New tests cover padded organized input, reordered/unaligned fields, both byte
orders, stable ties, exact XYZ/ring preservation, compact metadata, all-invalid
and empty clouds, nonfinite XYZ/intensity/time, negative/overflowing point times,
invalid headers, polar overflow, missing/duplicate/overlapping/out-of-bounds
fields, invalid datatypes/counts, short strides and truncated/extra payloads.
Original positive synthetic motion-compensation tests remain synthetic results.
The real three-scan script is a separate reproducible assertion-based test.

Evidence: [conversion_smoke.json](conversion_smoke.json),
[full smoke log](conversion_smoke.log),
[sensor CTest results](conversion_sensor_ctest.txt),
[deskew CTest results](conversion_deskew_ctest.txt).

## Reproduce without installing packages

Run from the Atlas repository. The existing `.closed_loop_cache/install` supplies
already-built pinned message dependencies on this VM. An equivalent sourced
dependency overlay can be used elsewhere. Builds are isolated and single-job.
Direct CMake was used because the local colcon coordinator stalled after its
sensor-package install completed; its compiled output was reused, not replaced.

```bash
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
cmake -S src/atlas_sensor_kit -B /tmp/atlas_velodyne_build/atlas_sensor_kit \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/tmp/atlas_velodyne_install/atlas_sensor_kit
cmake --build /tmp/atlas_velodyne_build/atlas_sensor_kit -j1
cmake --install /tmp/atlas_velodyne_build/atlas_sensor_kit
export AMENT_PREFIX_PATH=/tmp/atlas_velodyne_install/atlas_sensor_kit:${AMENT_PREFIX_PATH}
export CMAKE_PREFIX_PATH=/tmp/atlas_velodyne_install/atlas_sensor_kit:${CMAKE_PREFIX_PATH}
export LD_LIBRARY_PATH=/tmp/atlas_velodyne_install/atlas_sensor_kit/lib:${LD_LIBRARY_PATH}
cmake -S src/atlas_distortion_corrector_test \
  -B /tmp/atlas_velodyne_build/atlas_distortion_corrector_test \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DATLAS_REAL_GROUND_SMOKE=ON \
  -Datlas_sensor_kit_DIR=/tmp/atlas_velodyne_install/atlas_sensor_kit/share/atlas_sensor_kit/cmake
cmake --build /tmp/atlas_velodyne_build/atlas_distortion_corrector_test -j1
ctest --test-dir /tmp/atlas_velodyne_build/atlas_sensor_kit --output-on-failure
ctest --test-dir /tmp/atlas_velodyne_build/atlas_distortion_corrector_test --output-on-failure
/usr/bin/python3 scripts/smoke_velodyne_recording.py \
  /home/marian/Downloads/engg_loading_dock_01.db3 \
  --probe /tmp/atlas_velodyne_build/atlas_sensor_kit/real_cloud_smoke \
  --deskew-probe /tmp/atlas_velodyne_build/atlas_distortion_corrector_test/real_cloud_deskew_boundary \
  --ground-probe /tmp/atlas_velodyne_build/atlas_distortion_corrector_test/real_ground_smoke \
  --output /tmp/atlas_velodyne_smoke.json
```

Omit `--ground-probe` and leave `ATLAS_REAL_GROUND_SMOKE` OFF to build only the
existing deskew dependency closure. Omit both downstream probe arguments for
converter-only real-data validation. Every ROS wait and subprocess has a deadline.

## Remaining integration work

1. Confirm decoder time reference and measured LiDAR/IMU rotations/extrinsics for
   this recording. The adapter makes its interpretation explicit but cannot
   certify capture-time semantics.
2. Recover synchronized linear velocity/twist; the three IMUs do not replace it.
   Then test full component deskew with verified TF and acquisition timestamps.
3. Establish vehicle/gravity frame and geometry before vehicle-frame ground
   filtering. Validate physical ground classifications against reference labels.
4. Establish a defensible map-frame initial pose and independent reference before
   seeded NDT replay or any localization accuracy claims.
5. Validate native bag playback, `/clock` and cross-process transport in a normal
   ROS environment with local socket access.

No source bag or pinned upstream source was changed. No system packages were
installed, existing demos edited, extrinsics invented, commits made or pushes
performed. Existing uncommitted root README and demo work were preserved.
