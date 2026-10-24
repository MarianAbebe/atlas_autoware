# Engineering loading dock recording audit

Audited 2026-10-21 on ROS 2 Humble / ARM64. **Result: readable real sensor
messages and plausible replay timestamps; incompatible preprocessing input.
The bounded real-data contract probe correctly rejected the recorded cloud.
No successful deskew or NDT replay, and no localization accuracy result.**

## Evidence and scope

Input: `/home/marian/Downloads/engg_loading_dock_01.db3`, opened SQLite read-only
with query-only mode and a 4 MiB page cache. Storage span is
496.513975552 s, starting at 1752868633000622336 ns (2025-07-18 UTC).
The bag schema identifies its producer as `rosbags`, schema version 4; it has
seven topics and no embedded metadata rows. ROS Humble successfully deserialized
its standard CDR PointCloud2 and Imu messages. This is not evidence that a Humble
rosbag2 player accepts every feature of this newer SQLite schema: native bag
playback was not tested. Velodyne packet deserialization/type availability was
not tested; the existing decoded point-cloud topic avoids that dependency.

[audit.json](audit.json) contains actual headers, fields, ranges, point examples
(nonfinite values represented as null), rates, counts and timestamps. Payload
sampling uses indexed one-second windows at start, middle and end: one cloud
and at most 128 IMUs per stream per window. Counts scan metadata columns across
the bag, without reading camera payloads. No complete replay, full-payload scan,
or large in-memory bag copy was performed.

| Topic | Count | Count / bag duration | Frame in sampled messages |
|---|---:|---:|---|
| `/velodyne_points` | 4,918 | 9.905 Hz | `lidar_vlp16_points_link` |
| `/velodyne_packets` | 4,918 | 9.905 Hz | not inspected |
| `/novatel/oem7/imu/data_raw` | 62,000 | 124.871 Hz | `imu_eg320_link` |
| `/mcu_node/data_link/icm/data` | 94,760 | 190.851 Hz | `imu_icm20948_link` |
| `/xsens/imu/data` | 198,400 | 399.586 Hz | `imu_mti680g_link` |
| `/cam_0/image_raw`, `/cam_1/image_raw` | 2,480 each | 4.995 Hz each | not inspected |

Count/duration includes topic startup/end coverage; sampled header interval
rates are approximately 125, 191.04–191.05 and 400 Hz respectively. All sampled
IMU headers increase within their windows. All sampled orientations are
(0,0,0,0) with zero orientation covariance: **no usable attitude measurement**.
Sample accelerations have z near +9.8 m/s²; this suggests upward-positive
sensor z at capture but does not establish alignment to vehicle axes. Angular
velocity and acceleration are populated. Calibration, covariance quality and
physical clock synchronization remain unverified.

## Point-cloud contract

All three samples: height 1824, width 16, 29,184 slots, 22-byte points,
352-byte rows, 642,048 data bytes, little endian, `is_dense=false`.

| Field | Offset | ROS datatype | Count |
|---|---:|---|---:|
| x, y, z | 0, 4, 8 | FLOAT32 (7) | 1 each |
| intensity | 12 | FLOAT32 (7) | 1 |
| ring | 16 | UINT16 (4) | 1 |
| time | 18 | FLOAT32 (7) | 1 |

First header: 1752868633 s + 17,206,000 ns. Middle and final-window headers:
1752868881 s + 334,336,000 ns and 1752869128 s + 542,008,000 ns.
Finite XYZ counts are 17,297 / 18,681 / 17,019; a converter must explicitly
handle invalid returns rather than propagate them as valid geometry.
First-cloud XYZ bounds are x [-85.999,79.896], y [-41.972,102.623],
z [-1.736,22.919]. These values are consistent with local metric LiDAR
coordinates, not georeferenced map coordinates. Neither the message layout nor
these bounds proves x-forward/y-left mounting or calibrated orientation.

**Per-point timing exists.** `time` spans 0 to approximately 0.100839 s,
consistent with scan-relative seconds for a ~10 Hz VLP-16. The first three
slots have ring 0/1/2 and time 0/4.608e-6/9.216e-6. Each sampled cloud has
1,824 time decreases in flattened storage order. Decoder provenance must
confirm units/reference and ordering before converting to relative UINT32
nanoseconds and sorting/reorganizing for the deskewer. Simply renaming `time`
is insufficient.

Atlas `LivoxToExtendedComponent` requires an unorganized 26-byte seven-field
Livox cloud with absolute FLOAT64 nanosecond timestamps. This recording fails
organization, stride, fields and time encoding. `ExtendedToCompactComponent`
requires unorganized 32-byte XYZIRCAEDT. The pinned distortion corrector checks
for `time_stamp` and the XYZIRCAEDT layout, neither present here. NDT's ability
to consume XYZ does not remove the transform and pose requirements.

## Replay time

Sample header minus storage timestamp differences are within [-128,+128] ns
(across the inspected cloud and IMU samples), consistent with rounded storage
of the same epoch. There is no sampled multi-second epoch mismatch. Replaying
storage time as `/clock` and setting `use_sim_time=true` on every time-dependent
consumer is therefore temporally plausible. Keep headers and per-point times
in the same original epoch; do not restamp only one stream. Pause/seek clock
jumps require resetting motion/pose buffers. No `/clock` is recorded; the player
must generate it. Global monotonicity, dropouts, acquisition latency and actual
sensor synchronization were not established by these windows. A timestamp
match in a converted `rosbags` file cannot establish hardware synchronization.

## Campus map and calibration evidence

The existing campus demo uses Downloads `pointcloud_map_georeferenced.pcd`:
1,004,344 finite XYZ points, binary PCD, fields x/y/z plus four padding bytes,
16-byte stride. Bounds (m): x [-79.56354,122.83969],
y [-76.59005,119.27671], z [-5.39756,21.73112]. SHA-256:
`6121b6bf8df4324ab42286822f0e6f00c67ecc10daecf84d180129a6069ef9a3`.
Projector YAML SHA-256:
`614af74d6aaefe052c435338b585797a451e7922efc290329374c2abd2b8992d`.
Both match the existing map validation report; no map reload was necessary.

Projector: LocalCartesian / WGS84, origin latitude 51.08080935188234,
longitude -114.1331143659806. Metadata describes a custom height origin
1094.7826704429463 m, ground roughly -2.05 m, while pinned projection loading
uses origin altitude zero. Existing documentation requires an altitude bridge
for GNSS initialization. The projector is not a vehicle initial pose.
The YAML explicitly labels the package experimental and describes a restored
map after a failed scale correction. Historical fit residuals are not ground
truth or a fresh validation of this recording.

Related artifacts exist under
`~/roam/references/autoware_launch/maps/eng_loading_dock_map` and
`~/roam/references/autoware_launch/sensor_kit/eng_loading_dock_sensor_kit`.
The sensor-kit YAML claims measured LiDAR translation (-0.5,0,2.05) m,
identity kit-to-base, and zero LiDAR rotation. However, its generation script
accepts lever arms and defaults yaw to zero; other sensor transforms include
zero placeholders. Earlier mapping notes explicitly call extrinsics unknown.
These artifacts are useful candidates, **not sufficient proof of the complete
recording-specific calibration**. They were not imported into Atlas.

Historical mapping used a GNSS-enriched bag named `engg_loading_dock_01_gnss`
on another machine; this bag has no GNSS. A historical KISS-to-ENU fit and
initialization scripts were found, but the scripts depend on live GNSS/heading
and twist, with an identity-yaw fallback. No chain tying a timestamped pose,
its frame, lever arms and verified orientation to this exact bag was established.
No such fallback was used. Even if the map was built from the same traversal,
self-matching would not provide independent localization accuracy evidence.

## Bounded real-data test

[contract_smoke.txt](contract_smoke.txt) records the successful **negative
contract test**, using the first real cloud's original serialized CDR.
A small offline C++ executable compiles the unchanged Atlas conversion source,
deserializes through ROS serialization, and calls both existing conversions.
Both reject with `expected unorganized cloud (height=1)`. Exit status 0 means
both expected rejections occurred; it does not mean preprocessing succeeded.
No replacement converter, identity extrinsic, zero-motion sample or pose seed
was introduced. This test does not exercise DDS, `/clock`, component loading,
upstream deskew or NDT. No cloud reached those downstream stages.

No successful preprocessing replay is currently possible through the existing
input contract without an integration change. NDT was not attempted because
of that incompatibility and the absent defensible pose/calibration chain.
Existing synthetic results remain separate and were not rerun or reclassified.

## Prioritized integration steps

1. Confirm the VLP-16 decoder's `time` semantics, invalid-return policy and
   ring semantics; implement a separate Velodyne-to-XYZIRCAEDT adapter with
   explicit chronological ordering, finite-point handling, nanosecond bounds,
   intensity conversion and honest unknown-return metadata. Preserve the Livox
   adapter. Start with a few real scans and transport/layout assertions.
2. Obtain recording-specific measured LiDAR-to-base translation **and rotation**,
   vehicle axis definition and IMU-to-base rotations. Reconcile the historical
   YAML claims and defaults; publish verified static TF only.
3. Recover synchronized velocity/odometry or GNSS-enriched source data. Three
   gyro streams alone do not supply vehicle linear velocity for full deskew.
   Do not interpret unchanged output with absent motion as successful correction.
4. Establish a map-frame initial pose (including yaw, height, timestamp and
   uncertainty) through verified GNSS/heading plus projector/altitude handling,
   surveyed correspondences, or a documented map-referenced alignment. Wire the
   NDT map loader/service and timestamped pose-buffer inputs. No pose/TF/odom,
   GNSS fix/heading, camera calibration/CameraInfo, or independent reference is
   present among the seven recorded topics.
5. Test a bounded native ROS playback segment with `/clock` after checking player
   schema support; an isolated small derivative bag may be needed if unsupported.
   Keep the original immutable. Then run short deskew and seeded NDT checks with
   bounded memory, finite outputs, timing and convergence diagnostics. Report
   execution/convergence separately from accuracy; accuracy needs an independent
   reference. Cameras are not required for this LiDAR/NDT smoke test.

## Reproduction

From `~/roam/atlas_autoware`, with existing ROS installation only:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 scripts/audit_recording.py \
  /home/marian/Downloads/engg_loading_dock_01.db3 \
  --cloud-cdr /tmp/atlas_real_cloud.cdr > /tmp/atlas_real_audit.json
cmake -S scripts/real_data_contract -B /tmp/atlas_real_contract_build
cmake --build /tmp/atlas_real_contract_build -j1
/tmp/atlas_real_contract_build/contract_smoke /tmp/atlas_real_cloud.cdr
```

Only one 642 KiB cloud is exported when requested; no cameras are copied.
The audit script is deliberately strict about the inspected Velodyne layout.
The C++ probe is a source-level contract test without optional Autoware point-type
helpers, not a substitute for the existing component integration regressions.

Inspected pinned versions: Autoware root tag 1.9.0; Core
`f25f83c632c1984ec276c894c41857d4abc0dad8`; Universe
`6e477c645efec33f7909095eea684474e97f5e3d`. The root remained clean.
No bag/upstream changes, package installations, demo edits, commits or pushes.
Previously uncommitted README and demo directories were preserved.
