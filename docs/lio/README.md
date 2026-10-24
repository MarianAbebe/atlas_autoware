# Atlas LiDAR-inertial odometry integration

This is an integration boundary for FAST-LIO2, not a substitute estimator. It selects the ROS 2 wrapper at `https://github.com/Hliudi/FAST-LIO`, pinned to `2659eb81addb01ee0d6b8998f6e73154b684b8ab`; it wraps the FAST-LIO2 IESKF/ikd-tree core. Canonical HKU-MARS FAST-LIO is upstream ROS 1, so this is a third-party ROS 2 port and must be qualified independently.

The stack is headless and isolated from actuators:

```text
atlas_sensor_sim PointCloud2(x,y,z,intensity,time) -> atlas_livox_adapter -> Livox CustomMsg -> fastlio2_ros2 -> /atlas/lio/odometry
atlas_sensor_sim Imu ----------------------------------------------------------^ 
ground_truth -> evaluator only (never FAST-LIO or adapter)
```

`time` is seconds from scan start and is converted losslessly enough for the fixture to Livox `offset_time` nanoseconds. The simulator now creates a fixed, non-planar world in instantaneous LiDAR coordinates, ordered scan points, 10 Hz scans, and 200 Hz IMU. It includes translation and yaw motion but not true rolling shutter/motion distortion within a scan.

Install the exact external dependency in a separate workspace; do not vendor or silently upgrade it:

```bash
source /opt/ros/humble/setup.bash
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
