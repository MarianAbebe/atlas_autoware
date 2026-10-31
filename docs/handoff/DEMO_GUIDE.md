# Saturday simulation demonstration guide

This guide runs the existing validated ROS 2 simulation and uses RViz as a
separate viewer. It does not add a TF publisher or change the localization,
planner, controller, watchdog, or validator.

## Before the demo

On the Ubuntu desktop terminal, verify `DISPLAY` is set and that `rviz2` opens
with the installed Qt/OpenGL stack. Run `tools/native_ubuntu/preflight.sh` with
`ATLAS_MAP_DIR` set to the directory containing the three map assets. The
current verified asset location on the development host is `/home/marian/Downloads`;
replace it with the supplied asset directory on another machine. The preflight
checks checksums and reports RAM, disk, and graphics-tool availability.

Open a terminal in the repository and source the established ROS/Autoware
environment:

```bash
cd ~/roam/atlas_autoware
export ATLAS_MAP_DIR=/home/marian/Downloads
source src/atlas_full_stack_demo/scripts/environment.sh
ros2 pkg prefix atlas_full_stack_demo
ros2 pkg prefix atlas_map_integration_demo
```

Because this checkout may have an older cached `scene.py`, create a fresh
focused overlay before demonstrating the current source. Use unique paths on
each machine/run:

```bash
DEMO_BUILD="/tmp/atlas_demo_build_$(date -u +%Y%m%d_%H%M%S)"
DEMO_INSTALL="/tmp/atlas_demo_install_$(date -u +%Y%m%d_%H%M%S)"
source src/atlas_full_stack_demo/scripts/environment.sh
cmake -S src/atlas_full_stack_demo -B "$DEMO_BUILD" \
  -DCMAKE_INSTALL_PREFIX="$DEMO_INSTALL"
cmake --build "$DEMO_BUILD" --parallel 1
cmake --install "$DEMO_BUILD"
SAMPLE_BUILD="${DEMO_BUILD}_sample_vehicle"
SAMPLE_DESCRIPTION="$ATLAS_AUTOWARE_ROOT/src/core/autoware_core/description/autoware_sample_vehicle_description"
cmake -S "$SAMPLE_DESCRIPTION" -B "$SAMPLE_BUILD" \
  -DCMAKE_INSTALL_PREFIX="$DEMO_INSTALL"
cmake --build "$SAMPLE_BUILD" --parallel 1
cmake --install "$SAMPLE_BUILD"
export ATLAS_DEMO_INSTALL="$DEMO_INSTALL"
export AMENT_PREFIX_PATH="$DEMO_INSTALL${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
printf 'Use this overlay in every demo terminal: %s\n' "$ATLAS_DEMO_INSTALL"
```

This builds only the Atlas demo package and the existing Autoware sample
vehicle description against installed overlays. The first terminal retains the
exported `ATLAS_DEMO_INSTALL` variable for the clear and obstacle runs.

`environment.sh` selects ROS domain 86. Use the same sourced environment in
each terminal so RViz and the demo join the same graph. Do not start another
simulation in that domain. The full-stack launch keeps its map and TF ownership;
RViz is only a subscriber.

## Clear-route run

In the same sourced terminal used for the focused build, start one bounded
validation run (the validator's existing acceptance criteria are unchanged):

```bash
export ATLAS_MAP_DIR=/home/marian/Downloads
RESULTS="/tmp/atlas_demo_clear_$(date -u +%Y%m%d_%H%M%S)"
python3 src/atlas_full_stack_demo/scripts/run_validation.py \
  --scenario clear --timeout 60 --output "$RESULTS"
```

For a live, non-validator presentation that leaves the launch running until
Ctrl+C, use the supported full-stack launch instead:

```bash
RESULTS="/tmp/atlas_demo_live_clear_$(date -u +%Y%m%d_%H%M%S)"
ros2 launch atlas_full_stack_demo full_stack.launch.py \
  obstacle:=false request_engagement:=true results_dir:="$RESULTS"
```

Use `obstacle:=true` for the obstacle-stop presentation. The validator command
is preferred when a machine-readable PASS/FAIL result is required.

Terminal 2, start the low-load full-stack view:

```bash
source src/atlas_full_stack_demo/scripts/environment.sh
export ATLAS_DEMO_INSTALL=/tmp/atlas_demo_install_<printed-suffix>
export AMENT_PREFIX_PATH="$ATLAS_DEMO_INSTALL${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
ros2 launch atlas_full_stack_demo demo_visualization.launch.py
```

Use the same literal install path printed by the build step, replacing
`<printed-suffix>` with its generated suffix.

This visualization-only launch starts `robot_state_publisher` with the
description-only one-link URDF plus RViz. It starts no simulator and publishes
no transform edge. The sample model root is `base_link`, so RViz places it using
the full stack's existing `map -> base_link` TF. The mesh comes from the pinned
Autoware 1.9.0 sample vehicle package. It is a recognizable Lexus stand-in, not
measured Atlas body geometry.

The resource-limited view uses campus lanelet road markers, clustered obstacle
points, route, planner trajectory, route endpoints, EKF odometry, and the ego
model. For a clear run, raw LiDAR can be enabled by passing
`rviz_config:="$ATLAS_DEMO_INSTALL/share/atlas_full_stack_demo/config/demo_full_stack.rviz"`;
that preset displays raw LiDAR and clusters and may be heavier. The original
~1-million-point campus PCD is deliberately omitted. RViz has no built-in
display for the stack's `PredictedObjects` contract, so clustered points are the
visible perception output. The selected route is from the real map while the
scene supplies synthetic motion and LiDAR geometry in the route's map
coordinates.

## Obstacle-stop run

After the clear run has exited, use fresh results and launch exactly one new run:

```bash
source src/atlas_full_stack_demo/scripts/environment.sh
export ATLAS_MAP_DIR=/home/marian/Downloads
export AMENT_PREFIX_PATH="$ATLAS_DEMO_INSTALL${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
RESULTS="/tmp/atlas_demo_obstacle_$(date -u +%Y%m%d_%H%M%S)"
python3 src/atlas_full_stack_demo/scripts/run_validation.py \
  --scenario obstacle --timeout 60 --output "$RESULTS"
```

Start `demo_visualization.launch.py` in another sourced terminal with the same
`ATLAS_DEMO_INSTALL` overlay. The validator evidence directory records detections, plans, commands,
localization, freshness, scene timing, and stop/clearance checks. Keep the RViz
cluster display enabled to see the clustered obstacle return. A passing
obstacle validator is the safety evidence; a visible point cluster alone does
not establish a planner stop.

## FAST-LIO2 pose view (separate demonstration)

FAST-LIO2 remains isolated from NDT/EKF, planning, and control. Run its own
synthetic launch in a separate terminal/session and ROS domain according to
`docs/lio/README.md`; do not run it concurrently in domain 86 with the full-stack
demo. In a terminal sourced for that same FAST-LIO2 domain:

```bash
rviz2 -d "$ATLAS_REPO_ROOT/src/atlas_lio_integration/config/fastlio_pose_shadow.rviz"
```

This view uses fixed frame `odom` and only subscribes to the validated
pose-only shadow topic. It does not imply a `map -> odom` transform, EKF fusion,
or downstream autonomy readiness. Do not enable a second TF owner to make
frames appear connected.

## Startup checks and fallback

Before each run, check the configured domain and required topics:

```bash
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID DISPLAY=$DISPLAY"
ros2 topic list --no-daemon
ros2 topic info /atlas_loc/ekf/ekf_odom --no-daemon --spin-time 2
ros2 topic info /atlas_loc/lidar/raw --no-daemon --spin-time 2
ros2 topic info /atlas_loc/route_path --no-daemon --spin-time 2
ros2 topic info /atlas_loc/trajectory_visualization --no-daemon --spin-time 2
```

In RViz, `Global Status` must show Fixed Frame `map` without a transform error.
If a display says `No transform`, verify the message frame and the existing TF
tree (`ros2 run tf2_ros tf2_echo map base_link` and
`ros2 run tf2_ros tf2_echo base_link synthetic_lidar`). Do not change fixed
frames to hide a missing transform. For performance, first disable the raw
cloud, then the cluster cloud; keep map roads, route, trajectory, and EKF pose.
If GUI rendering remains unreliable, run the validator headlessly and show its
JSON result and recorded topic summaries. That fallback proves software checks,
not a live visual demonstration.

## Current verification and limits

On the development host, RViz and the RobotModel launched with OpenGL 4.5. The
resource-limited obstacle view passed 28/28 validation checks, with 2.09 m
clearance and no watchdog latch. The captured RViz view rendered around 9 fps.
A clear-route headless run passed 26/26 with 0.121 m final goal error and no
watchdog latch. Two runs using both raw and clustered clouds in the heavier RViz
view latched stale NDT; the resource-limited view with one cloud and the sample
vehicle passed. This points to visualization/resource pressure on this 3 GiB
ARM64 VM as a likely contributor, not a proven cause.

The generic Lexus is a visualization stand-in, not measured Atlas geometry.
`glxinfo` is unavailable, so the precise GPU vendor/renderer is unidentified;
RViz did initialize OpenGL 4.5. These synthetic results do not establish
hardware readiness.
