# Atlas Nav2 skid-steer demo

This package is the first independently runnable PRD-aligned navigation
milestone. It connects a deterministic 2D map and laser simulation to the ROS 2
Humble Nav2 planner/controller and routes Nav2 velocity commands through a
default-disabled simulation safety gate, four-wheel mixer, and ideal skid-steer
kinematic simulator.

It is separate from the established Autoware prototype and does not source or
build Autoware. It never connects to a physical motor controller.

## Dependencies

On Ubuntu 22.04 with ROS 2 Humble installed:

```bash
sudo apt-get update
sudo apt-get install --no-upgrade --no-remove \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander python3-pytest
```

Review the package transaction before approving it. The demo uses the binary
Humble Nav2 release and only builds `atlas_nav2_demo` locally.

## Build and test

```bash
cd ~/atlas_autoware
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/atlas_nav2_demo/test
tools/nav2/build.sh
```

The build uses one job and writes ignored artifacts to `.nav2_cache`.

## Launch

```bash
cd ~/atlas_autoware
source /opt/ros/humble/setup.bash
source .nav2_cache/install/setup.bash
ros2 launch atlas_nav2_demo nav2_skid_steer.launch.py
```

The gate starts disabled. For manual simulation testing only, explicitly enable
it after Nav2 becomes active:

```bash
ros2 service call /safety_gate/enable std_srvs/srv/SetBool '{data: true}'
```

Disable it before shutdown:

```bash
ros2 service call /safety_gate/enable std_srvs/srv/SetBool '{data: false}'
```

The automated validator handles enable, goal submission, zero-command checks,
disable, evidence capture, and cleanup:

```bash
tools/nav2/run_validation.sh
```

## Configuration

- Simulation geometry: `config/simulation.yaml`. `wheel_radius` and
  `track_width` must match in the mixer and simulator. Defaults are provisional
  test geometry, not measured chassis dimensions.
- Nav2: `config/nav2_params.yaml`.
- Map: `maps/synthetic_map.yaml`, an 8.0 m by 6.0 m, 0.10 m/cell occupancy map
  with boundary walls and a central rectangular obstacle.
- Frames: `map -> odom -> base_link -> laser`.

Important topics:

| Topic | Type | Purpose |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | Synthetic static map |
| `/atlas_nav2/scan` | `sensor_msgs/LaserScan` | Deterministic ray-cast sensor |
| `/atlas_nav2/cmd_vel_nav` | `geometry_msgs/Twist` | Ungated Nav2 output |
| `/atlas_nav2/cmd_vel_safe` | `geometry_msgs/Twist` | Safety-gated simulation command |
| `/atlas_nav2/wheel_speed_targets` | `sensor_msgs/JointState` | Four angular-velocity targets |
| `/atlas_nav2/wheel_states` | `sensor_msgs/JointState` | Ideal encoder position/velocity feedback |
| `/atlas_nav2/odom` | `nav_msgs/Odometry` | Simulated odometry feedback |
| `/atlas_nav2/safety_status` | `std_msgs/String` | JSON gate state and freshness |

## Troubleshooting

- `Missing nav2_*`: install the listed Humble packages; do not add Autoware
  overlays to solve a Nav2 dependency.
- Lifecycle nodes remain inactive: inspect the launch log in the validator output
  directory and verify `map_server`, planner, controller, behavior server, and BT
  navigator parameters.
- No motion: confirm the simulation gate was explicitly enabled and that
  `/atlas_nav2/cmd_vel_nav` is fresh. Never increase the 250 ms gate timeout to
  hide scheduling or discovery failures.
- No local obstacles: verify `/atlas_nav2/scan` uses frame `laser`, the TF chain
  exists, and the local obstacle layer subscribes to that exact topic.
- A failed validator run is not a pass. Preserve its directory, inspect
  `summary.json` and `launch.log`, and rerun only after fixing the cause.
