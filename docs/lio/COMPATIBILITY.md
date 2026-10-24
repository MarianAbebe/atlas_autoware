# Compatibility assessment

| Area | FAST-LIO2 decision |
|---|---|
| Algorithm | FAST-LIO2 direct scan-to-map, IESKF, incremental kd-tree; chosen over LIO-SAM because no feature-extraction pipeline is required for Livox-style solid-state scans. |
| ROS 2 Humble | Canonical `hku-mars/FAST_LIO` documents ROS 1 prerequisites. Selected `Hliudi/FAST-LIO` is a third-party Humble wrapper, pinned at `2659eb81…`; upstream does not endorse it. |
| ARM64 / Orin | Canonical upstream documents ARM platforms, but no Atlas/Orin benchmark is claimed. The selected wrapper uses CPU/OpenMP; tune thread count, map cube, voxel filtering and RSS on the 16 GB Orin. |
| Livox Mid-360S | Mid-360 packet protocol has timestamps and equally spaced points. FAST-LIO Livox mode consumes `CustomMsg` offsets. Actual Mid-360S driver/firmware compatibility is unvalidated. |
| Input | `sensor_msgs/Imu` in rad/s and m/s², monotonic common time base; Livox `CustomMsg` containing XYZ, reflectivity, line and `offset_time` ns. |
| Extrinsics | `lidar_link`/`imu_link` rigid transform and time offset are provisional identity/zero only. Field calibration is mandatory. |
| GTSAM | FAST-LIO odometry can become a factor input, but covariance, reset/loop-closure semantics, and factors need a dedicated adapter; GTSAM is not linked here. |

The target’s GPS/PTP time source, actual LiDAR-to-IMU mounting transform, IMU noise/bias characterization, Livox line/tag behavior, high-rate point load, thermal performance, and behavior through motion distortion require hardware validation.
