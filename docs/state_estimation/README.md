# Atlas state-estimation interface foundation

This milestone is a headless, deterministic ROS 2 Humble sensor fixture. It is independent of Autoware and Nav2. It produces measurements plus an evaluation-only `/atlas/sensors/evaluation/ground_truth`; no estimator-facing node subscribes to that topic, and it publishes no estimated odometry.

```text
synthetic trajectory -> LiDAR / IMU / GNSS / wheel encoders / stereo -> future estimators
                    `-> evaluation/ground_truth (test use only)
all sensor topics -> health monitor -> /atlas/sensors/health
```

On Ubuntu 22.04 with ROS 2 Humble:

```bash
cd ~/atlas_autoware
source /opt/ros/humble/setup.bash
colcon build --packages-select atlas_sensor_sim --symlink-install --executor sequential
source install/setup.bash
ros2 launch atlas_sensor_sim sensor_stack.launch.py
ros2 run atlas_sensor_sim validate_sensor_stack.py
```

Use `params_file:=/absolute/path/to/your.yaml` to override defaults. The fixed `scenario:=deterministic` is intentionally lightweight. It does not control hardware or motor topics.

Synthetic clouds and imagery are small by design. The images have feature texture and fixed horizontal disparity, but do not reproduce IMX296 optics, exposure, rolling shutter, lens distortion, or real synchronization. GNSS uses a local tangent-plane approximation valid only over small areas; it is not a NovAtel GNSS/INS model.

Future deployment requires a Pi 5 acquisition graph, Jetson/Orin performance validation, actual drivers, calibrated extrinsics, common time synchronization, and hardware fault testing.
