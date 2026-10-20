# Atlas synthetic vehicle motion demo

This test-only package composes the pinned Autoware
`autoware::simulator::simple_planning_simulator::SimplePlanningSimulator`
component with the `IDEAL_STEER_ACC` model. It does not start a physical
vehicle driver or a hardware interface.

The simulator and demo node use the `/atlas_vehicle_sim` namespace. TF uses
the unique test frames `atlas_sim_odom` and `atlas_sim_base_link`. The simulator
publishes odometry, velocity, steering, gear, control-mode status and TF; the
demo node publishes a temporary RViz marker using synthetic body dimensions.

All geometry in `config/simulator.param.yaml` is provisional visualization and
simulation data only. It is not measured Atlas geometry or calibration. The
simulator uses a synthetic 1.20 m wheelbase, a 0.45 rad steer limit, and a
1.80 x 1.00 x 0.50 m body marker.

The bounded run waits for command discovery, odometry, vehicle-status topics
and TF, then applies forward acceleration, a positive steering angle, and
braking. It measures acceleration and braking from timestamped velocity
reports and asserts steering response, displacement, heading change, final
speed, gear/mode reports, odometry timestamps/frame IDs, speed agreement,
and odometry-to-TF agreement. The
launch shuts down when the test node exits.

After building this package and the pinned simulator in an isolated overlay,
source the completed overlays and run:

```bash
source /opt/ros/humble/setup.bash
source /tmp/atlas_deskew_deps_install/setup.bash
source /tmp/atlas_vehicle_sim_install/setup.bash
source /tmp/atlas_vehicle_motion_demo_install/setup.bash
ros2 launch atlas_vehicle_motion_demo vehicle_motion_demo.launch.py
```

To launch RViz with an eight-second post-test hold for viewing the settled
vehicle, run:

```bash
ros2 launch atlas_vehicle_motion_demo vehicle_motion_demo.launch.py launch_rviz:=true visualization_hold_sec:=8.0
```

The test prints numerical acceleration, braking, speed, steering,
displacement, heading change, timestamps, and TF error on success.
The test executable returns 1 on failure; check its exit and `FAIL:` output
because the launch command can exit successfully after shutting down a
failed test process.

See [VALIDATION.md](VALIDATION.md) for the latest build and run results.
