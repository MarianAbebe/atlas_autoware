# Sensor contracts and estimator boundary

The machine-readable contract is [sensor_contracts.yaml](../../config/sensor_contracts.yaml). Frames follow REP-103/105. `map` is ENU; `base_link` is forward-left-up; camera optical frames are right-down-forward. All header timestamps are ROS acquisition time and must be monotonic for each normal stream. SI units apply.

The simulator's defaults are explicitly provisional. Hardware adapters must preserve these interface meanings rather than silently treating synthetic calibration as field calibration.

| Consumer | Required boundary |
|---|---|
| FAST-LIO2 / LIO-SAM | `PointCloud2` with finite XYZ, `Imu`, calibrated `lidar_link`/`imu_link` extrinsics, compatible timestamps; a driver adapter may be needed for Livox point time/ring fields. |
| Stereo VINS-Fusion | matched `Image`/`CameraInfo`, `Imu`, optical frames, stereo intrinsics/baseline and camera-IMU calibration; real image transport/encoding adapter may be needed. |
| NovAtel GNSS/INS | `NavSatFix` is only a basic adapter boundary; integrate the vendor driver for solution status, covariance, velocity, heading, PPS/time and lever arm. |
| GTSAM | consume calibrated, time-aligned measurements and health state; factors, outlier handling, and graph implementation are intentionally absent. |
| Nav2 | only a future, validated estimator output (`nav_msgs/Odometry` plus REP-105 transforms) may feed localization. This fixture does not provide one. |

`/evaluation/ground_truth` is for test scoring only and is prohibited as an estimator input. The existing Nav2 ideal localization remains a separate regression fixture, not sensor-derived localization.
