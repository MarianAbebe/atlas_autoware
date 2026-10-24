# Stationary full-stack quick start

> This procedure is for the already-built ARM64 reference VM. For a new native
> Ubuntu laptop, use [NATIVE_UBUNTU_TESTING.md](NATIVE_UBUNTU_TESTING.md); it
> documents source import, external map transfer and fresh-workspace build.

These commands are verified against the repository as it exists on the reference
VM. They intentionally initialize localization without engaging the simulator.

## 1. Supported reference environment

- Ubuntu 22.04 ARM64 under UTM on macOS
- ROS 2 Humble, default Fast DDS RMW
- Six virtual CPUs, approximately 4 GiB RAM
- Repository: `~/roam/atlas_autoware`
- Pinned Autoware source: `~/roam/references/autoware` (or set by `ATLAS_AUTOWARE_ROOT`)
- ROS domain: 86, set by `environment.sh`

A clean-clone build on another machine has not been validated. See
[REPOSITORY_AUDIT.md](REPOSITORY_AUDIT.md) before attempting one.

## 2. Required local inputs

The current launch requires these exact map assets. They are not bundled in Git.
Transfer them to a local directory (e.g., `~/atlas_assets`) and export
`ATLAS_MAP_DIR`.

| File | SHA-256 |
|---|---|
| `finalMap_repaired.osm` | `1e9360ea62b709271007d27672120b3d0040ebabc6667d646de68734b1702fee` |
| `map_projector_info.yaml` | `614af74d6aaefe052c435338b585797a451e7922efc290329374c2abd2b8992d` |
| `pointcloud_map_georeferenced.pcd` | `6121b6bf8df4324ab42286822f0e6f00c67ecc10daecf84d180129a6069ef9a3` |

Verify without modifying them:

```bash
export ATLAS_MAP_DIR="$HOME/atlas_assets"
sha256sum \
  "$ATLAS_MAP_DIR/finalMap_repaired.osm" \
  "$ATLAS_MAP_DIR/map_projector_info.yaml" \
  "$ATLAS_MAP_DIR/pointcloud_map_georeferenced.pcd"
```

The build also requires the existing `.closed_loop_cache`, `.obstacle_cache`,
`.perception_cache`, and `.localization_cache` overlays. They are machine-local
artifacts, not a substitute for a dependency manifest.

The authoritative direct dependency list is
[`src/atlas_full_stack_demo/package.xml`](../../src/atlas_full_stack_demo/package.xml).
It includes the Atlas localization/map/perception packages, Autoware ground
filter, clustering/planning components, ROS message packages, `rclpy`,
`sensor_msgs_py`, `tf2_ros_py`, NumPy, component support and rosbag. There is no
validated clean-machine package-install command for the complete stack; do not
infer one from the current caches.

## 3. Confirm no graph is already running

From the repository root:

```bash
cd ~/roam/atlas_autoware
source src/atlas_full_stack_demo/scripts/environment.sh
ros2 node list --no-daemon --spin-time 3
ps -eo pid,ppid,stat,comm,args | rg \
  'full_stack.launch.py|scene.py|localization_watchdog.py|perception_guard.py|autoware_ndt_scan_matcher_node|component_container'
```

If Atlas nodes already exist, find their owning launch terminal and stop that
launch with Ctrl+C. Do not start a second instance and do not terminate unknown
processes blindly.

## 4. Build

The established full-stack build verifies the four pinned revisions, uses one
compiler job, refreshes the localization/full-stack overlays, and runs the
adapter and scheduler-telemetry unit tests:

```bash
cd ~/roam/atlas_autoware
bash src/atlas_full_stack_demo/scripts/build_demo.sh
```

It assumes the prerequisite overlays already exist. It does not install system
packages. On 2026-10-23 this workflow completed with four static-object tests and
three scheduler-telemetry tests passing.

## 5. Source the runtime environment

Run this in every terminal:

```bash
cd ~/roam/atlas_autoware
source src/atlas_full_stack_demo/scripts/environment.sh
```

Confirm package resolution:

```bash
ros2 pkg prefix atlas_full_stack_demo
ros2 pkg executables atlas_full_stack_demo
```

The expected prefix is under `.full_stack_cache/install`.

## 6. Launch while disengaged

Use the clear scene for the stationary handoff:

```bash
RESULTS=/tmp/atlas_handoff_stationary_$(date -u +%Y%m%d_%H%M%S)
echo "$RESULTS"
ros2 launch atlas_full_stack_demo full_stack.launch.py \
  obstacle:=false request_engagement:=false results_dir:="$RESULTS"
```

The explicit `request_engagement:=false` keeps the simulator disengaged. Do not
omit it. Do not run `manual_start.py`; that helper
publishes to `/atlas_loc/sim/input/engage` after initialization.

## 7. Initialize localization without engagement

In a second sourced terminal:

```bash
ros2 run atlas_full_stack_demo initialize_localization.py
```

The helper refuses to proceed if the simulator is moving faster than 0.001 m/s
or the combined watchdog is latched. It activates EKF and NDT, waits for the
bootstrap subscriber, publishes one map-frame initial pose, and waits for both
NDT and EKF output. It never publishes an engagement message.

Expected final line:

```text
Localization initialized successfully: post-bootstrap NDT and EKF outputs observed.
```

## 8. Verify topics and TF

Use `--no-daemon --spin-time 3` for graph queries on this VM:

```bash
ros2 topic info /atlas_loc/lidar/compact --verbose --no-daemon --spin-time 3
ros2 topic info /atlas_loc/ndt/ndt_pose_with_covariance --verbose --no-daemon --spin-time 3
ros2 topic info /atlas_loc/ekf/ekf_odom --verbose --no-daemon --spin-time 3
ros2 topic echo /atlas_loc/ndt/ndt_pose_with_covariance --once --field header
ros2 topic echo /atlas_loc/ekf/ekf_odom --once --field header
ros2 topic echo /atlas_loc/trajectory --once --field header
```

Expected nominal rates are approximately 10 Hz compact LiDAR, 10 Hz NDT, and
50 Hz EKF. Measure one stream at a time and stop `ros2 topic hz` with Ctrl+C:

```bash
ros2 topic hz /atlas_loc/lidar/compact
ros2 topic hz /atlas_loc/ndt/ndt_pose_with_covariance
ros2 topic hz /atlas_loc/ekf/ekf_odom
```

Before localization initialization, a dynamic `map` to vehicle transform may be
absent. After successful initialization, inspect:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link synthetic_lidar
```

Stop each command with Ctrl+C after receiving transforms.

## 9. Verify safety

```bash
ros2 topic echo /atlas_loc/base_safety_status --once
ros2 topic echo /atlas_loc/safety_status --once
ros2 topic echo /atlas_loc/sim/output/odometry --once
```

After the one-second healthy arming interval, both JSON status messages must show:

```text
"armed": true
"latched": false
"currently_stale": []
```

Odometry linear velocity must remain zero. If either monitor latches, preserve
the evidence and stop the experiment. Do not rearm or increase the threshold.

## 10. Clean shutdown

1. Stop any inspection commands with Ctrl+C.
2. Stop an optional telemetry process with Ctrl+C.
3. Stop an optional rosbag recorder with Ctrl+C and wait for finalization.
4. Stop the full-stack launch with Ctrl+C.
5. Confirm no Atlas nodes remain with `ros2 node list --no-daemon --spin-time 3`.

The stationary procedure never engages the simulator, so no disengagement command
is necessary.
