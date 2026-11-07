# State-estimation integration boundaries

This document defines integration contracts; it does not claim that FAST-LIO2,
LIO-SAM, VINS-Fusion, a NovAtel receiver, or GTSAM is running on Atlas hardware.
The machine-readable sensor contract is `config/sensor_contracts.yaml`.

## Source selection and TF ownership

Exactly one selected estimator publishes the authoritative dynamic
`odom -> base_link` transform and `nav_msgs/Odometry` for a deployment mode.
Exactly one selected global-localization source publishes `map -> odom`.
Sensor drivers publish only calibrated static `base_link -> sensor` transforms.
No estimator may consume `/atlas/sensors/evaluation/ground_truth`; that topic is
test scoring only. Launch files must select one localization mode, never launch
NDT/EKF, FAST-LIO2, LIO-SAM, VINS-Fusion, and a fusion graph as competing TF
authorities.

Every production adapter must reject its input when its source timestamp is
invalid, future, duplicate/regressing, non-finite, wrong-frame, or stale.
Receipt age and source age must be retained separately. An unhealthy selected
source must cause status degradation and the safety/command gate to stop; it
must not be replaced by simulator truth or a refreshed timestamp. The current
synthetic health monitors validate only the fields they document and are not a
substitute for those deployed adapter checks.

## FAST-LIO2 / LIO-SAM

`atlas_lio_integration/livox_adapter.py` is the existing FAST-LIO2 boundary. It
maps `PointCloud2{x,y,z,time}` to Livox `CustomMsg`, preserves the cloud header
as `timebase`, and converts per-point offsets to `offset_time` nanoseconds. It
expects `/atlas/sensors/lidar/points` in `lidar_link` and
`/atlas/sensors/imu/data` in `imu_link`. Hardware integration additionally
requires measured LiDAR-to-IMU extrinsics, clock synchronization, driver field
mapping, covariance conventions, and actual evaluation. The synthetic launch is
only usable when the separately pinned `fastlio2_ros2` package is installed.

LIO-SAM may use the same input contract but needs a dedicated adapter selected
by launch configuration; it must not run beside FAST-LIO2 as a TF publisher.

## VINS-Fusion

VINS-Fusion receives matched left/right `Image` plus `CameraInfo` streams and
the IMU stream in the frames and acquisition-time convention in
`sensor_contracts.yaml`. Deployment prerequisites are camera intrinsics,
stereo baseline, camera-to-IMU extrinsics, image encoding, and synchronization
limits. There is no VINS-Fusion runtime implementation in this repository.

## NovAtel GNSS/INS

`NavSatFix` is only an initial ROS boundary. A real PwrPak7D-E2 driver adapter
must also publish solution/fix type, position/orientation/velocity covariance,
heading convention, lever-arm-corrected frame, PPS or receiver time provenance,
and invalid-fix status. Invalid or degraded fixes cannot update global fusion or
publish `map -> odom` as though they were valid.

## GTSAM fusion

No GTSAM factor graph is implemented. A future graph must explicitly define
state variables (pose, velocity, IMU bias, and optional clock/lever-arm terms),
IMU preintegration, LiDAR/VIO relative-pose factors, GNSS position/heading
factors, wheel factors, covariances, robust outlier policy, reset policy, and
health output. Wrapping the existing EKF would not satisfy this requirement.

## Current testable evidence

`atlas_sensor_sim` supplies deterministic synthetic LiDAR, IMU, GNSS, wheel,
and stereo contracts with health monitoring. `atlas_lio_integration` tests
adapter/evaluation timestamp and frame rejection behavior. Neither is hardware
validation or localization-accuracy evidence.
