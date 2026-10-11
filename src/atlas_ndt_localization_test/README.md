# Synthetic pinned Autoware NDT demonstration

This test-only package loads the registered `autoware::ndt_scan_matcher::NDTScanMatcher`
factory from the installed pinned upstream shared library. It compiles no replacement
registration algorithm. Upstream solver parameters and convergence thresholds are
loaded unchanged from `ndt_scan_matcher.param.yaml`; only test frame names and loaded-map
visualization are overridden. Intra-process transport is disabled.

The upstream `test/test_util.hpp` generates the matched three-face, 20 m geometry and
1 m downsampled scan. The unchanged `test/stub_pcd_loader.hpp` serves the 30,603-point
map translated to (100,100,0), through `/pcd_loader_service`. This mock is deliberately
test-only. No hardware calibration or geographic placement is implied.

The demonstration activates NDT, waits for discovery, static TF and actual loaded-map
publication, then supplies two timestamped pose seeds before each scan. Seeds are
deliberately biased by (+0.3,+0.2,0) m. Ground-truth positions are (100,100,0) and
(101,100.5,0), with identity orientation. The second scan is translated inversely to
the simulated vehicle movement; map geometry stays fixed. Assertions require:

- Position error <0.10 m and orientation error <0.02 rad at both positions.
- Estimated displacement within 0.10 m of (+1,+0.5,0) m in x and y.
- Correct map, pose/scan frames, timestamps, aligned point counts and estimated TF.
- Exactly one registered NDT factory, with real ROS topic and service communication.

The map frame is `map`. Identity static TF relates `atlas_ndt_test_base` and
`atlas_ndt_test_lidar`. Synthetic ground-truth TF `map -> atlas_ndt_test_base` is
published only to display input scans; NDT publishes its independent
`map -> atlas_ndt_test_estimate` transform. A full EKF is unnecessary for this
seeded synthetic test; this does not establish complete vehicle localization.

The full pipeline case dynamically loads the two Atlas converter factories, the
registered pinned distortion corrector and the registered NDT component. All four
communicate through ROS 2 topics with intra-process transport disabled. The upstream
corrector receives explicit base-frame, IMU-disabled, 2D-only and polar-update-disabled
parameters. Constant synthetic forward motion is 1 m/s over each 50 ms acquisition.
Livox absolute point timestamps are generated in nanoseconds, aligned to the 256 ns
FLOAT64 epoch precision at this date. The test verifies exact relative point offsets,
XYZIRCAEDT and XYZIRC fields, metadata, preserved headers and deskew displacement.
NDT subscribes to the final compact topic, so its pose and aligned cloud are only
produced after that transport path delivers the processed scan.

## Isolated build

Prerequisites: ROS Humble, the previously prepared debug-message prefix, and approved
`ros-humble-diagnostic-updater` / `ros-humble-geographic-msgs` installations. No command
below installs software or edits upstream source. Run from the Atlas repository.

```bash
source /opt/ros/humble/setup.bash
source /tmp/atlas_deskew_deps_install/setup.bash
export ENABLE_AGNOCAST=0
export MAKEFLAGS="-j2 -l2"
export ROS_LOCALHOST_ONLY=1
colcon --log-base /tmp/atlas_ndt_log build \
  --base-paths ~/roam/references/autoware/src/core \
  --packages-ignore autoware_lint_common autoware_utils_system \
  --packages-select autoware_ndt_scan_matcher autoware_cmake \
    autoware_agnocast_wrapper autoware_internal_localization_msgs \
    autoware_localization_util autoware_map_msgs autoware_qos_utils \
    autoware_utils_diagnostics autoware_utils_logging autoware_utils_pcl \
    autoware_utils_visualization autoware_utils_rclcpp autoware_common_msgs \
    autoware_utils_geometry autoware_utils_math autoware_utils_tf \
    autoware_internal_planning_msgs autoware_perception_msgs autoware_planning_msgs \
  --build-base /tmp/atlas_ndt_build --install-base /tmp/atlas_ndt_install \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source /tmp/atlas_ndt_install/setup.bash
source /tmp/atlas_deskew_install/setup.bash
colcon --log-base /tmp/atlas_ndt_demo_log build \
  --base-paths src/atlas_ndt_localization_test \
  --build-base /tmp/atlas_ndt_demo_build --install-base /tmp/atlas_ndt_demo_install \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source /tmp/atlas_ndt_demo_install/setup.bash
colcon --log-base /tmp/atlas_ndt_demo_log test \
  --base-paths src/atlas_ndt_localization_test \
  --build-base /tmp/atlas_ndt_demo_build --install-base /tmp/atlas_ndt_demo_install \
  --event-handlers console_direct+
colcon test-result --test-result-base /tmp/atlas_ndt_demo_build --verbose
```

`ATLAS_NDT_SOURCE` is a CMake cache path to the pinned upstream package, defaulting to
`~/roam/references/autoware/src/core/autoware_core/localization/autoware_ndt_scan_matcher`.
The mock loader occupies a global service name; run only one demo per ROS domain.
Every discovery, response and output wait is bounded; CTest adds a 90-second process
timeout. No output means failure, rather than an assumed pose estimate.
Use a normal terminal with localhost socket access. In the restricted agent sandbox,
colcon stalled after CMake exited; the same build completed outside that sandbox.

## RViz

Source the same prefixes in both terminals and use the same ROS domain/RMW settings.
Run RViz first, then the demo, which alternates between the two positions for a
bounded 120-second viewing interval after initial validation:

```bash
rviz2 -d src/atlas_ndt_localization_test/config/ndt_demo.rviz
# In another terminal:
ros2 run atlas_ndt_localization_test ndt_synthetic_demo --ros-args \
  -p pipeline:=true -p hold_seconds:=120
```

Use the second command for the full LiDAR-to-NDT pipeline. The saved configuration
displays the loaded map, Livox input, compact cloud delivered to NDT, aligned scan,
estimated pose and test TF in `map`. Hold time is restricted to 0–600 seconds
(default 0 for CTest).

## Validation on Ubuntu 22.04 ARM64 / Humble

The 19 pinned source packages built successfully with upstream tests and Agnocast
disabled. Colcon discovery excludes the two source test-only packages listed above;
otherwise colcon requests their missing install markers even with BUILD_TESTING=OFF.
The debug messages are taken from the existing isolated dependency overlay.
Autoware Core source revision: `f25f83c632c1984ec276c894c41857d4abc0dad8`.

Both two-position CTests passed: direct synthetic NDT and the full four-component
LiDAR-to-NDT chain (2 tests, 0 failures). Numerical results from the full pipeline:

| Ground truth (m) | NDT result (m, rounded) | 3D position error |
|---|---|---|
| (100,100,0) | (100.003,100.003,0.00272513) | 0.00461343 m |
| (101,100.5,0) | (101.003,100.503,0.00276485) | 0.00458508 m |

Each 1,261-point XYZIRC cloud had 16-byte points. Point timing and metadata were
verified across the chain, and deskew restored approximately 0.05 m. Both orientation
checks, estimated displacement, pose timestamps, aligned scan counts and estimated
TF checks passed without changing the preset assertions.
NDT published the 30,603-point loaded map and loaded its registered factory
from `/tmp/atlas_ndt_install/autoware_ndt_scan_matcher/lib/libautoware_ndt_scan_matcher.so`.
CTest output is in `/tmp/atlas_ndt_demo_build/atlas_ndt_localization_test/Testing/Temporary/LastTest.log`.
An initial TF preflight can warn that a frame is not yet present, before the static
transform arrives; the readiness check must succeed before any scan is sent.

RViz opened with the saved configuration and remained running during a 10-second
alternating-position run of the full pipeline (exit code 0). Both logs contain no ROS
error lines: `/tmp/atlas_ndt_visualization_chain/{rviz,demo}.log`. This validates
configuration loading and live topic traffic; it is not a screenshot-based review of
display appearance.

Existing regressions also passed: 9 converter GTests, 4 deskewer GTests and 4
three-component chain GTests. Colcon reports 27 entries for those 17 GTests because
it also counts 10 CTest wrappers. To reproduce the regression run:

```bash
source /opt/ros/humble/setup.bash
source /tmp/atlas_deskew_deps_install/setup.bash
source /tmp/atlas_deskew_install/setup.bash
export ROS_LOCALHOST_ONLY=1
colcon --log-base /tmp/atlas_ndt_regression_log test \
  --base-paths src/atlas_sensor_kit src/atlas_distortion_corrector_test \
  --build-base /tmp/atlas_deskew_build --install-base /tmp/atlas_deskew_install \
  --event-handlers console_direct+
colcon test-result --test-result-base /tmp/atlas_deskew_build --verbose
```

This milestone does not validate recorded scans, real-world map quality, capture
timestamps, physical TF calibration, unseeded initialization or an EKF feedback loop.
