# Atlas Ackermann 3D showcase

The showcase is an RViz visualization layer over the authoritative isolated
Ackermann simulation. It does not publish odometry, vehicle commands, scan
data, or navigation goals. The simulator remains the sole `odom -> base_link`
authority and Nav2 still reaches the vehicle only through `safety_gate`.

## Run

```bash
source /opt/ros/humble/setup.bash
source ~/roam/diagnostic_overlays/atlas_ackermann_v1/install/setup.bash
export ROS_DOMAIN_ID=211
LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 ros2 launch atlas_nav2_demo ackermann_showcase.launch.py scenario:=open
```

Use `scenario:=avoidable` or `scenario:=blocked` for matching barrier/cone
layouts. For a resource-only graph check without RViz, use `rviz:=false`.

`atlas_rover_showcase.urdf` is repository-authored primitive geometry rather
than a claim about Atlas hardware. Its wheelbase, wheel positions, body size,
sensor placements, and camera/IMU mounts are provisional visual values. The
wheel and steering joints are driven from actual `/atlas_ackermann/control_cmd`;
the entire RobotModel follows the simulator's `odom -> base_link` TF.

Course barrier markers are generated from `world.py`, which is also used by the
ray-cast LaserScan and simulator collision check. Trees, grass, and road are
non-colliding decorative markers outside the navigable obstacle rectangles;
they never represent clearance constraints.

## RViz rendering modes

The default `rviz/ackermann_showcase.rviz` is the lightweight VM view. It
renders the rover, course markers, scan, path, and TF, but omits Map displays.
This avoids RViz's indexed 8-bit map shader path, which has been reported to
raise `active samplers with a different type refer to the same texture image
unit` on the llvmpipe VM. The view is capped at 8 Hz and starts framed around
the rover and course. Navigation map and costmap topics remain active and are
unchanged.

The original RViz config containing map/costmap displays is retained as
`rviz/ackermann_showcase_full.rviz` for GPU-capable systems. The full config
has not been visually verified on this VM.

Diagnostics for the 2026-11-07 investigation are in
`~/roam/validation_evidence/ackermann_showcase/debug/`. Default and custom
RViz both initialized OpenGL 4.5 with llvmpipe without reproducing the shader
error in their startup logs. In the full graph, `ackermann_simulator` was the
single odometry publisher; the two `/tf` publishers were the simulator and
`robot_state_publisher`, which owns the visual wheel/sensor child frames.
Course MarkerArray, joint states, and URDF description were received. RViz
used about 233 MiB RSS and ranged near 7–89% CPU in brief observations; the VM
had roughly 124–137 MiB free RAM with swap already in use. These short samples
indicate software-rendering load but do not prove it caused the black view.

X11 root screenshot capture returned `BadMatch` in this VM session, so visual
appearance remains unverified here. To check locally, verify the Display panel
shows no red status for RobotModel, Outdoor course, or TF, then inspect the
3D view. The initial camera is aimed at `map` around `(-0.2, 0, 0.55)`.

## Asset policy and optional Gazebo

No downloaded asset is redistributed by this repository. This avoids unclear
community-model licenses and keeps the showcase small enough for the 4 GB VM.
Gazebo Fuel can be used on a stronger machine, but each model has its own
license metadata and must be reviewed before downloading or redistributing.
The NASA/JPL Open Source Rover is Apache-2.0 but is a six-wheel rover, so it
was not used as an Atlas drivetrain representation.

Gazebo is intentionally optional and was not installed. Check the transaction
before any installation:

```bash
apt-get -s install ros-humble-ros-gz-sim
```

Gazebo must subscribe to the existing state for visualization only; it must not
run a second vehicle physics, odometry, TF, or command graph.
