# atlas_distortion_corrector_test

Test-only Humble ARM64 harness for the unchanged pinned Autoware distortion corrector.
No sensor configuration or replacement deskew implementation is included.

The library compiles the upstream distortion corrector, node, memory utilities,
managed transform buffer/provider, and math trigonometry/sin table sources directly.
Pinned compatibility utility headers and point types are included directly; only
`autoware_internal_debug_msgs` requires a separately built pinned message package.
`ATLAS_AUTOWARE_REFERENCE` selects the reference checkout (defaults to
`$HOME/roam/references/autoware`). Keep that checkout at the verified pinned versions.

Dependencies: Humble C++/components/TF/messages, Sophus, Eigen, PCL common,
point_cloud_msg_wrapper, pcl_ros, pinned debug messages, ament GTest. No full Autoware build.

```bash
source /opt/ros/humble/setup.bash
source /tmp/atlas_deskew_deps_install/setup.bash
colcon --log-base /tmp/atlas_deskew_log build --packages-select atlas_sensor_kit atlas_distortion_corrector_test \
  --build-base /tmp/atlas_deskew_build --install-base /tmp/atlas_deskew_install \
  --cmake-args -DBUILD_TESTING=ON
colcon --log-base /tmp/atlas_deskew_log test --packages-select atlas_sensor_kit atlas_distortion_corrector_test \
  --build-base /tmp/atlas_deskew_build --install-base /tmp/atlas_deskew_install
colcon test-result --test-result-base /tmp/atlas_deskew_build
```

Each CTest scenario is a separate process (isolated motion queues and managed TF
singleton). Localhost ROS domain 178, namespace `/atlas_deskew_test`, and dedicated
synthetic frames isolate the graph. Intra-process communication is disabled.
The registered upstream factory is dynamically loaded. Reliable depth-100 motion
is acknowledged before best-effort cloud publication. Discovery/output deadlines
are five seconds, TF preflight and acknowledgment two seconds, CTest thirty seconds.
Cloud delivery retries every 100 ms to tolerate best-effort startup loss.

Identity static TF is checked through the same managed-TF provider before any cloud
is sent. Missing TF is not exercised: upstream failure paths read uninitialized
transform storage. IMU, 3D correction and polar updates are disabled; all six node
parameters are supplied explicitly.

Two points at `(1,0,0)` have header time 100 s and offsets 0/50 ms.
Forward 1 m/s motion expects x=1/1.05 m; zero, missing and stale (99.5 s) motion
expect unchanged XYZ. All tests check exact header/layout/count/ordering/timestamps,
metadata and every non-XYZ byte, with XYZ tolerance 1e-5 m.

Synthetic time is deliberately independent of wall time; latency diagnostics may
warn and are not assertions of hardware timing. Missing/stale inputs can publish
unchanged clouds; output alone does not establish healthy correction. No IMU,
rotation, real extrinsics, sensor clocks, DDS bandwidth or hardware is validated.
Run through CTest/colcon rather than multiple scenarios in one process.

## Validation

Validated on Ubuntu 22.04 ARM64, ROS 2 Humble, with Sophus 1.22.9102,
point_cloud_msg_wrapper 1.0.7, pcl_ros 2.4.5 and pinned debug messages 1.13.0.
The initial missing pcl_ros dependency is now declared explicitly. The plugin
library contains the unchanged node source and is loaded dynamically; a separate
shared support library contains the unchanged algorithm/utilities/managed TF.
The executable links only support, avoiding duplicate plugin registration while
sharing the managed-TF singleton with the component.

Results are recorded under the isolated build's `test_results`, `deskew_*.txt`,
and `chain_*.txt`. Socket-restricted sandboxes may produce DDS transport errors;
run with localhost socket access for validation.

## End-to-end chain

The installed `atlas_sensor_kit` plugin is located through its CMake package prefix
and loaded with class_loader, never linked into the executable. Its real factories
are `atlas_sensor_kit::LivoxToExtendedComponent` and
`atlas_sensor_kit::ExtendedToCompactComponent`; the third factory is the pinned
`autoware::pointcloud_preprocessor::DistortionCorrectorComponent`.
All three use private PointCloud2 input/output ports with SensorDataQoS.
Test-only topic wiring in `/atlas_deskew_test` is:

```text
input (Livox) -> LivoxToExtended -> extended (XYZIRCAEDT)
  -> DistortionCorrector -> rectified (XYZIRCAEDT)
  -> ExtendedToCompact -> output (XYZIRC)
twist (TwistWithCovarianceStamped) -> DistortionCorrector
```

Four additional CTest processes cover forward, zero, missing and stale motion
through the full chain. The Livox cloud has the verified seven fields/26-byte
stride, header 100 s and absolute FLOAT64 nanosecond times 100000000000 and
100050000000. Intensities 21.5 and 87.25 must become 22 and 87. Distinct tag/line
values must not populate Autoware metadata: channel and return_type remain zero.
Intermediate subscriptions check conversion offsets and all preserved bytes
after correction. The final cloud must have exactly six fields, 16-byte stride
and 32 bytes of data; each point equals the corrected extended cloud's first
16 bytes, with expected XYZ and preserved header/order/intensity/metadata.
No conversion function or substitute algorithm is called by the test.

The original four deskewer tests remain registered. Each runs in its own process;
both test groups share the same TF and motion delivery safety checks.

## Latest end-to-end validation

Both required Atlas packages built successfully in the isolated prefix.
All four end-to-end scenarios passed; the original four deskewer scenarios and
eight converter unit tests plus one converter ROS integration test also passed.
Total: 17 distinct GTest cases, zero failures/errors/skips. Colcon reports 27
entries because it also counts the ten CTest wrappers. All ten test logs were
checked for duplicate plugin registration and DDS transport errors: none found.
No upstream or production Atlas implementation was changed.
