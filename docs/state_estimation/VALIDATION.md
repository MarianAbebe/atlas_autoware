# Validation

Focused static/unit validation:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/atlas_sensor_sim/test
colcon build --packages-select atlas_sensor_sim --symlink-install --executor sequential
source install/setup.bash
ros2 launch atlas_sensor_sim sensor_stack.launch.py > /tmp/atlas_sensor_stack.log 2>&1 & stack_pid=$!
ros2 run atlas_sensor_sim validate_sensor_stack.py | tee /tmp/atlas_sensor_validation.json
kill -TERM "$stack_pid"
```

The validator emits machine-readable JSON `PASS`/`FAIL` and checks required streams, timestamps, finite LiDAR indication, IMU validity, plausible GNSS, stereo synchronization, and diagnostic publication. Evidence is normally `/tmp/atlas_sensor_validation.json` and `/tmp/atlas_sensor_stack.log`.

Fault scenarios use parameter overrides and retain deterministic seeds: `gnss_dropout_probability: 1.0`, `lidar_dropout: true`, non-zero `timestamp_offset_sec`, `camera_frame_loss_probability: 1.0`, or `encoder_noise_rad_s: 0.1`. The health monitor transitions missing/stale inputs to ERROR (and invalid/fresh inputs to WARN/ERROR as applicable). Test harnesses must expect failure for deliberately absent mandatory topics. Fault behavior is unit-tested for deterministic geometry/noise; live health loss is covered by launching with `lidar_dropout: true` and observing an ERROR diagnostic.

This is not a vehicle safety supervisor, sensor qualification, estimator accuracy validation, or physical-hardware test. No LIO, VIO, GTSAM, NovAtel integration, or sensor-derived Nav2 localization has been implemented.
