# Atlas simulator validation, 2026-10-11 / 2026-10-12 UTC

The pinned `autoware_simple_planning_simulator` 1.0.0 checkout at
`82d2c93a28a25f8802e8f5a21b4ec850beea9080` built successfully without upstream
edits. The test uses `IDEAL_STEER_ACC` with provisional synthetic geometry;
these results do not validate measured Atlas vehicle dynamics.

## Dependency audit and build

Before resuming compilation, the 24-package dependency closure and the Atlas
demo were checked with the existing rosdep database for build, buildtool,
export, and runtime dependencies. All system dependencies were satisfied.
An additional check resolved 482 production include references in the three
unfinished packages and checked 73 cached shared libraries with `ldd`.
No unresolved headers or shared libraries remained. No packages were installed.

The cached build skipped 21 successfully built packages. Fresh CMake caches
were used for the remaining Lanelet2 utilities, motion utilities, and simulator;
all three finished successfully. Build parallelism was limited to two compiler
jobs with sequential package execution. The Atlas demo built in a separate
overlay. Upstream stderr contained compiler ABI notes and a CMake header-install
advisory, with no build failures in the resumed simulator build.

Build and audit evidence:

- `/tmp/atlas_vehicle_sim_log/latest_build/`
- `/tmp/atlas_vehicle_sim_dependency_audit.txt`
- `/tmp/atlas_vehicle_sim_header_audit.txt`
- `/tmp/atlas_vehicle_sim_closure.tsv`
- `/tmp/atlas_vehicle_motion_demo_log/latest_build/`

## Bounded motion test

The scenario accelerates at +0.8 m/s² for approximately 0.95 seconds,
steers at +0.35 rad, brakes at -0.8 m/s², then clears both commands.
The executable verifies motion after approximately 3.16 seconds, with a
15-second readiness/scenario timeout and a separate 25-second launch bound.

The first successful bounded run returned 0 and produced:

| Measurement | Result |
| --- | --- |
| Measured acceleration | +0.800 m/s² |
| Measured braking | -0.800 m/s² |
| Peak forward speed | 0.752 m/s |
| Displacement x / y | 1.509 / 0.210 m |
| Displacement magnitude | 1.523 m |
| Heading change | 0.359 rad |
| Final velocity report / odometry speed | 0.000 / 0.000 m/s |
| Peak / final steering report | 0.350 / 0.000 rad |
| Gear / control mode | DRIVE / AUTONOMOUS |
| Odometry samples at verification | 159 |
| Odometry timestamp | 1791752828.860373974 s |
| TF timestamp | 1791752828.860454798 s |
| TF position / yaw error | 0.00000 m / 0.00000 rad (printed precision) |

Odometry and velocity timestamps advanced monotonically. Vehicle status
timestamps were nonzero. Odometry frame IDs were `atlas_sim_odom` and
`atlas_sim_base_link`; velocity and odometry speed agreed within 0.03 m/s.
Displacement exceeded 1 m, heading change fell within 0.25–0.70 rad, and TF
pose agreement passed the 0.03 m / 0.03 rad tolerances.

Evidence: `/tmp/atlas_vehicle_motion_results/bounded_motion.log`.

## RViz demonstration

The actual desktop RViz process launched successfully, initialized OpenGL 4.5,
subscribed to `/atlas_vehicle_sim/visualization/vehicle`, and added a third
anonymous TF listener alongside the simulator and test listeners. Marker
messages had the expected frame, timestamps, cube type, and 1.80 × 1.00 × 0.50 m
dimensions. Acceleration reports reached +0.800000012 and -0.800000012 m/s².
All launched processes exited cleanly after a 12-second visualization hold.

In the final GUI run, motion verification reported displacement (1.507, 0.209) m,
magnitude 1.521 m, heading change 0.359 rad, peak speed 0.752 m/s, final speed
-0.016 m/s, steering 0.350 → 0.000 rad, and zero printed TF pose error.
The small final speed residual is allowed by the 0.15 m/s stop tolerance.
This ideal acceleration model integrates signed acceleration without clamping
velocity at zero, so a timer-sized braking overshoot can produce reverse drift
during the visualization hold.

Runtime, graph, and message checks passed. Rendered image inspection remains
unverified: the Xwayland image was black and GNOME's screenshot API returned
`AccessDenied: ScreenshotWindow is not allowed`. The external validator therefore
returned 1 for its stricter requirement that a screenshot also succeed; this was
not a motion-test or RViz process failure. No valid screenshot is claimed.

Evidence:

- `/tmp/atlas_vehicle_motion_results/rviz_motion.log`
- `/tmp/atlas_vehicle_motion_results/rviz_validation.json`

## Failures found and corrected in the local Atlas demo

1. Quaternion conversion link error: added the installed `tf2_geometry_msgs`
   dependency and its conversion header.
2. ROS could not discover the installed package: added its `ament_cmake`
   build-type export.
3. Humble launch rejected a bare shutdown event in `OnProcessExit`: wrapped it
   in an `EmitEvent` action.
4. RViz did not subscribe to the marker: changed the configuration property
   from `Marker Topic` to Humble's `Topic`.
5. External RViz validation initially assumed TF subscriptions used RViz's main
   node name; the listener actually uses an anonymous `transform_listener_impl`
   node. The graph check was corrected.

The Atlas test also gained numerical acceleration/braking checks and logging,
steering response verification, odometry speed agreement, and a body marker
offset that rotates with the simulated vehicle heading.

No upstream Autoware modifications, package installations, commits, or pushes
were performed.
