# Nav2 skid-steer validation

## Acceptance workflow

Run in order:

```bash
cd ~/atlas_autoware
source /opt/ros/humble/setup.bash
bash -n tools/nav2/build.sh tools/nav2/run_validation.sh
python3 -m py_compile src/atlas_nav2_demo/atlas_nav2_demo/*.py \
  src/atlas_nav2_demo/launch/*.py src/atlas_nav2_demo/scripts/*.py
python3 -m pytest -q src/atlas_nav2_demo/test
tools/nav2/build.sh
tools/nav2/run_validation.sh
```

The bounded validator requires active map, planner, controller, behavior, and BT
navigator lifecycle nodes; verifies map/costmap messages and the complete TF
chain; proves the gate is initially disabled; explicitly enables only the
simulation gate; submits a NavigateToPose goal; and requires nonzero Nav2,
gated, and wheel commands plus odometry displacement and goal approach.

The default acceptance goal is in open map space. It requires active Nav2
nodes, a map and both costmaps, the TF chain, explicit enablement of the
simulation-only gate, nonzero Nav2/gated/wheel commands, simulated odometry
motion, goal approach, and zero output after completion and disablement. The
central obstacle and scan source are present, but obstacle avoidance/replanning
is not an acceptance result until a dedicated blocked-route test passes.

Each run creates `summary.json`, `launch.log`, and per-process ROS logs in a new
output directory. A timeout, rejected goal, inactive lifecycle node, missing TF,
missing costmap, safety failure, or failed motion assertion produces a nonzero
exit status and `FAIL`; it must not be reported as a pass.

## Current evidence

The passing bounded run on the ARM64 development VM is recorded in
`/tmp/atlas_nav2_validation_short/summary.json` and `launch.log`. It completed
the navigation action in 8.56 seconds, moved 0.90 m, and ended 0.18 m from the
`(0.0, 1.0)` goal. It observed 84 Nav2 commands, 603 gated commands, 602
wheel-target messages, and 442 odometry messages. This validates the idealized
closed-loop simulation only; it is not evidence of obstacle avoidance or
physical driving. The shorter open-space goal reduces VM scheduling sensitivity;
its motion threshold is 0.7 m.

## Deterministic obstacle scenarios

Run the focused scenarios after building and sourcing the overlay:

```bash
ros2 run atlas_nav2_demo validate_closed_loop.py --scenario avoidable \
  --output /tmp/atlas_nav2_avoidable
ros2 run atlas_nav2_demo validate_closed_loop.py --scenario blocked \
  --output /tmp/atlas_nav2_blocked
```

`avoidable` uses a 0.50 m-wide rectangle on the direct route to `(4.5, 0.0)`.
The shared geometry is used by the map, LiDAR, collision guard, and validator;
the validator requires LiDAR detection, non-empty local costmap obstacle cells,
a lateral detour, at least 0.02 m footprint clearance, and goal success.
The passing ARM64 run is `/tmp/atlas_nav2_avoidable_final`: it reached the goal
in 14.86 s with 0.272 m minimum clearance and a local-costmap peak of 456 cells.

`blocked` uses a wall connected to both map boundaries. It requires Nav2 to
reject or fail the goal, LiDAR/costmap detection, collision clearance, and a
zero safe command after failure. It additionally injects one simulation-only
fresh command and requires the gate to zero it after the 0.25 s freshness limit.
The passing ARM64 run is `/tmp/atlas_nav2_blocked_pass_2`: Nav2 returned status
6 (aborted), the minimum clearance was 1.427 m, the costmap peak was 360 cells,
and stale output was zeroed. Recovery behavior moved 0.306 m laterally but did
not approach or cross the wall.

The earlier obstacle attempt was invalid because it checked only the final
rolling costmap grid, after the robot had left the obstacle, and the ideal
simulator did not enforce collision geometry. The new tests retain full
time-series costmap observation and prevent integration through obstacles.

## PRD alignment and remaining gaps

This milestone implements Nav2 planning/control interfaces and a four-wheel
skid-steer command/feedback simulation. It does not implement or validate GTSAM,
FAST-LIO2/LIO-SAM, VINS-Fusion, NovAtel, real Livox acquisition, camera
perception, physical motor control, wheel slip, or outdoor autonomy.
