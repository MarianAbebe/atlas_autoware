# Synthetic Autoware LiDAR perception demo

This test loads the registered pinned Autoware Core `EuclideanClusterNode` and
the three already-tested Atlas LiDAR preprocessing components. The actual
detector is CPU-based PCL Euclidean clustering. It accepts the Atlas 16-byte
XYZIRC result and publishes `autoware_perception_msgs/msg/DetectedObjects`.

The input consists of two synthetic 0.8 x 0.6 x 1.2 m point-grid obstacles,
each sampled at 0.1 m spacing in x/y and five heights. Their synthetic centers
are (6.0, 1.5, 0.8) m and (10.0, -2.5, 0.8) m in the test LiDAR frame. The
clusterer uses 2D connectivity (`use_height=false`) with a 0.22 m tolerance;
these are test settings and do not describe Atlas sensor geometry. A test-only
identity static transform joins distinct synthetic base and LiDAR frames, and
zero motion samples keep deskew deterministic. Livox point timestamps equal
the cloud timestamp, so the expected per-point offset is zero.

The program checks registered component factories, all transported cloud
layouts, exact message header times, two object centroids within 0.03 m of the
synthetic centers, `UNKNOWN` classification, stable results over three scans,
and the detector's colored cluster output. It publishes RViz sphere markers at
the detected centers; the marker size is a display glyph, not an estimated
obstacle bounding box. All waits are bounded.

Build the pinned Core component into an isolated prefix, then build and run the
Atlas test package:

```bash
source /opt/ros/humble/setup.bash
source /tmp/atlas_ndt_install/setup.bash
source /tmp/atlas_deskew_deps_install/setup.bash
source /tmp/atlas_deskew_install/setup.bash
source /tmp/atlas_lidar_perception_upstream_install/setup.bash
source /tmp/atlas_ndt_demo_install/setup.bash
export ENABLE_AGNOCAST=0 MAKEFLAGS="-j2 -l2" ROS_LOCALHOST_ONLY=1
colcon --log-base /tmp/atlas_lidar_perception_demo_log build \
  --base-paths src/atlas_lidar_perception_test \
  --build-base /tmp/atlas_lidar_perception_demo_build \
  --install-base /tmp/atlas_lidar_perception_demo_install \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source /tmp/atlas_lidar_perception_demo_install/setup.bash
colcon --log-base /tmp/atlas_lidar_perception_demo_log test \
  --base-paths src/atlas_lidar_perception_test \
  --build-base /tmp/atlas_lidar_perception_demo_build \
  --install-base /tmp/atlas_lidar_perception_demo_install \
  --event-handlers console_direct+
colcon test-result --test-result-base /tmp/atlas_lidar_perception_demo_build --verbose
```

RViz can display the synthetic Livox input, detector cluster cloud, and detected
center markers while the bounded demo is running:

```bash
rviz2 -d src/atlas_lidar_perception_test/config/lidar_perception.rviz
# In another sourced terminal:
ros2 run atlas_lidar_perception_test lidar_perception_demo --ros-args -p hold_seconds:=30
```

`hold_seconds` is bounded to 0–60 and defaults to 0 for automated tests. During
the hold interval, the demo keeps publishing new deterministic scans so RViz
can discover and display the point clouds and detected-center markers.

No physical sensor calibration or hardware-specific parameters are used.

## Recorded Velodyne perception

`real_lidar_perception_probe` (built with BUILD_TESTING) loads the separate
Velodyne converter, compact converter, pinned RANSAC component and pinned
EuclideanClusterNode. It accepts one actual cloud CDR and an output directory,
forwards the actual component outputs in bounded stages, and writes serialized
outputs plus steady-clock latency and Linux RSS measurements. It publishes no
motion, TF or simulator input. `ATLAS_RANSAC_LIBRARY` selects the already-built
pinned library from the previous real-data milestone.

The automated five-timestamp driver reads only small indexed cloud samples:

```bash
# Source the prefixes documented in the report, then:
/usr/bin/python3 scripts/validate_real_lidar_perception.py \
  /home/marian/Downloads/engg_loading_dock_01.db3 \
  --probe /tmp/atlas_real_perception_build/real_lidar_perception_probe \
  --output-dir /tmp/atlas_real_perception_results
```

Use a fresh output directory. Both sensor-z plane-filtered and unfiltered
clustering paths are tested with identical pinned clustering settings. Ground
orientation remains unverified. Real clusters are point groups, not confirmed
obstacles. The earlier synthetic test above is preserved as a separate CTest.

[Real-data results and reproduction](../../docs/real_data/engg_loading_dock_01/PERCEPTION.md)
include cluster sizes/centroids/bounds, measured latency/resources, and regression
status. No recorded detections are connected to the moving simulator.
