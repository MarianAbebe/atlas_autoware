# Atlas LiDAR-inertial odometry integration

This is an integration boundary for FAST-LIO2, not a substitute estimator. It selects the ROS 2 wrapper at `https://github.com/Hliudi/FAST-LIO`, pinned to `2659eb81addb01ee0d6b8998f6e73154b684b8ab`; it wraps the FAST-LIO2 IESKF/ikd-tree core. Canonical HKU-MARS FAST-LIO is upstream ROS 1, so this is a third-party ROS 2 port and must be qualified independently.

The stack is headless and isolated from actuators:

```text
atlas_sensor_sim PointCloud2(x,y,z,intensity,time) -> atlas_livox_adapter -> Livox CustomMsg -> fastlio2_ros2 -> /atlas/lio/odometry
atlas_sensor_sim Imu ----------------------------------------------------------^ 
ground_truth -> evaluator only (never FAST-LIO or adapter)
```

The scan header is the start time. Each `time` value is a per-point offset in seconds from that header and is converted to Livox `offset_time` nanoseconds. Points are reprojected from a static, non-planar landmark scene using the analytic vehicle pose at the assigned acquisition time. Scan offsets are ordered and fall within the configured 0.1 s period; IMU and evaluation ground truth use the same ROS epoch/monotonic elapsed-time mapping. FAST-LIO2 stamps its odometry at `lidar_end_time`, so evaluation interpolates ground truth at that estimator timestamp. This synthetic rotating scan approximates scan motion distortion; it does not model Mid-360 ray firing, beam divergence, visibility/occlusion, returns, line tags, noise statistics, or real clock synchronization.

The evaluator requires estimator `odom -> base_link` and ground-truth `map` frames. It interpolates truth at estimator header stamps using linear translation and quaternion SLERP, rejects extrapolation and brackets wider than the configured maximum, and rejects duplicate/regressing estimator stamps. One fixed SE(3) alignment from the first valid matched pose removes initial coordinate-frame offset. It reports aligned XY/Z/3D/yaw error plus consecutive relative-pose translation and rotation error; these relative metrics expose drift after alignment. Endpoint aligned position error is the last aligned 3D error, and drift percentage divides it by ground-truth path length. This is relative trajectory evaluation, not absolute map localization; no `map -> odom` transform is fabricated. Match rate is matched samples divided by unique finite, frame-valid estimator samples; duplicate callback receipts are separately counted and do not masquerade as missed unique measurements. Existing PASS limits remain XY RMSE <= 0.50 m, yaw RMSE <= 0.15 rad, at least 100 matched samples, nonzero output rate, and at least 90% unique-estimator match rate. Relative error metrics are reported but do not add unreviewed acceptance thresholds.

Install the exact external dependency in a separate workspace; do not vendor or silently upgrade it:

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install nlohmann-json3-dev libpcl-dev libeigen3-dev libyaml-cpp-dev
git clone https://github.com/Hliudi/FAST-LIO.git ~/atlas_fast_lio
cd ~/atlas_fast_lio && git checkout 2659eb81addb01ee0d6b8998f6e73154b684b8ab
colcon build --base-paths src --packages-select livox_interfaces fastlio2_ros2 --executor sequential --parallel-workers 1
source install/setup.bash
cd ~/atlas_autoware
colcon build --packages-select atlas_sensor_sim atlas_lio_integration --executor sequential --parallel-workers 1
source install/setup.bash
ros2 launch atlas_lio_integration fastlio_synthetic.launch.py
ros2 run atlas_lio_integration evaluate_lio
```

The external wrapper requires PCL, Eigen, yaml-cpp, nlohmann-json, OpenMP, and its `livox_interfaces` package. It expects Livox `CustomMsg`, not ordinary PointCloud2; `livox_adapter` is specifically for synthetic input. On hardware replace it with `livox_ros_driver2`/the selected production driver and retain its real point offsets.

The resulting contract is `nav_msgs/Odometry` at `/atlas/lio/odometry`, `odom` parent and `base_link` child frames. It must not be connected to Nav2 until a separate TF/localization integration review. `atlas_lio_health` reports stale LiDAR, IMU, or odometry only; it is not a vehicle safety supervisor.

## Pose-only shadow validation

FAST-LIO2 publishes nav_msgs/Odometry on /atlas/lio/odometry with odom parent
and base_link child frames. The wrapper's pose is the usable estimate; its
twist and covariance fields are not a calibrated integration contract.
fastlio_pose_shadow.launch.py runs only the isolated adapter. It copies the
original header stamp, odom frame, and pose into geometry_msgs/PoseStamped on
/atlas/lio/shadow/pose. It does not publish TF, twist, covariance, commands, or
messages to EKF/NDT/planning/control inputs. The existing strict odometry shadow
filter remains available for rejecting incomplete Odometry; it is not relaxed
to accept the wrapper's invalid fields.

Ground truth remains on /atlas/sensors/evaluation/ground_truth in map and is
captured separately. The exact-forwarding analyzer compares the complete
recorded unique ROS timestamp sets, frames, and pose values. Monotonic receipt
time is used only for capture overlap, rates, and receipt-gap metrics. Analysis
always writes to a new output directory and refuses to overwrite evidence.
Reported PASS is a forwarding-contract result, not localization accuracy or
hardware validation.

Build the focused package in a sourced ROS 2 Humble workspace:

    source /opt/ros/humble/setup.bash
    colcon build --base-paths src --packages-select atlas_lio_integration \
      --executor sequential --parallel-workers 1
    source install/setup.bash

For standalone synthetic validation, use separate terminals in one ROS domain:

    ros2 launch atlas_lio_integration fastlio_synthetic.launch.py
    ros2 launch atlas_lio_integration fastlio_pose_shadow.launch.py
    ros2 run atlas_lio_integration record_pose_shadow \
      --output-dir /tmp/atlas_pose_capture_20261026_01 --duration-sec 30
    ros2 run atlas_lio_integration analyze_pose_shadow \
      /tmp/atlas_pose_capture_20261026_01 \
      --output-dir /tmp/atlas_pose_capture_analysis_20261026_01

The output paths must not already exist. The synthetic launch resolves the
tracked config/fastlio_synthetic.yaml from the installed package by default
and fails if it is absent. It starts the sensor simulator, Livox format
adapter, FAST-LIO2, and LIO health monitor only. It does not start the Atlas
full-stack or any actuator path. The recorder writes four streams:
raw_estimator_odometry.jsonl, shadow_pose.jsonl,
shadow_pose_diagnostics.jsonl, and ground_truth.jsonl, plus
capture_metadata.json. Analysis output includes validation.json and
analysis_context.json.

The pose output is not EKF-ready: it contains no covariance, and odom is not
map. Do not connect it to localization fusion until a source-supported pose
uncertainty model and a defined map-to-odom ownership/alignment interface
exist. In particular, do not infer covariance from the wrapper's invalid
Odometry covariance array. FAST-LIO2's own odom-to-base_link transform is
confined to the standalone ROS domain; do not run it beside the existing Atlas
TF owner in a shared autonomy graph without a separate TF ownership design.
