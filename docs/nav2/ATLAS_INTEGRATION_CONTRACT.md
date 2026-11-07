# Atlas / Nav2 Integration Contract

Status: design only. This document specifies an isolated, mutually exclusive Nav2 mode for the Atlas synthetic simulation. It does not authorize runtime changes or hardware use. The development Autoware mode remains the validated fallback. No runtime files or safety parameters are changed by this design.

## 1. Architecture decision

Use two launch-time-exclusive modes:

- Autoware mode keeps the existing EKF, Autoware route/planning pipeline, MPC/PID controller, Autoware vehicle command gate, and simulator.
- Nav2 mode keeps only the selected Atlas simulation, localization, perception, and safety components needed by that mode. Nav2 owns global/local navigation planning and velocity control. An explicit adapter is the sole Nav2-mode producer of the Autoware simulator's accepted command type. The existing Autoware command gate remains the final command authority.

Never launch both controller stacks against the same plant. The current standalone Nav2 package remains useful as a dependency and algorithm baseline, but its own simulator and safety gate are not the Atlas full-stack plant or production safety system.

The provisional PDR-aligned requirements identify Nav2 planning as a target requirement, while the PRD traceability marks it not implemented in the operational Autoware graph. The official ROAM PDR still needs confirmation; see [PRD traceability](../handoff/PRD_V0_2_TRACEABILITY.md), [provisional requirements](../handoff/PDR_ALIGNED_REQUIREMENTS.md), and [PRD gap status](../handoff/PRD_GAP_CLOSURE_STATUS_2026-10-31.md).

## 2. Existing Atlas simulation interface

All relative scene and localization topics below resolve beneath `/atlas_loc`. The full-stack namespace and remappings are defined in [full_stack.launch.py](../../src/atlas_full_stack_demo/launch/full_stack.launch.py), [localization_loop.launch.py](../../src/atlas_localization_closed_loop_demo/launch/localization_loop.launch.py), and [scene.py](../../src/atlas_full_stack_demo/scripts/scene.py).

| Topic | Type | Publisher → consumer | Contract |
|---|---|---|---|
| `/atlas_loc/sim/output/odometry` | `nav_msgs/msg/Odometry` | Pinned SimplePlanningSimulator → scene sensor generator, watchdog stationary-state observation, validator | Simulation truth; parent `map`, child `hidden_truth_base` in the full-stack launch. Evaluation/sensor generation only; never Nav2 localization or control state. |
| `/atlas_loc/sim/output/steering` | `autoware_vehicle_msgs/msg/SteeringReport` | Simulator → base watchdog and controller feedback | Simulator-reported steering. |
| `/atlas_loc/sim/output/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | Simulator → scene; scene republishes as `/atlas_loc/sensors/acceleration` | Simulated body acceleration with acquisition stamp retained. |
| `/atlas_loc/sensors/twist` | `geometry_msgs/msg/TwistWithCovarianceStamped` | Scene → EKF | Simulated body twist; `base_link` frame and source measurement time. |
| `/atlas_loc/sensors/imu/data` | `sensor_msgs/msg/Imu` | Scene → configured sensor consumers | Synthetic IMU in `imu_link`; same ROS clock domain as the scene and simulator. |
| `/atlas_loc/lidar/raw` | `sensor_msgs/msg/PointCloud2` | Scene → Atlas format converter | Synthetic cloud in `synthetic_lidar`; scan start is the header stamp and each point carries its relative acquisition time. |
| `/atlas_loc/lidar/extended` | `sensor_msgs/msg/PointCloud2` | Livox-layout converter → deskew component | Converted cloud; acquisition stamp preserved. |
| `/atlas_loc/lidar/deskewed` | `sensor_msgs/msg/PointCloud2` | Deskew component → compact converter | Current synthetic configuration disables motion deskew. |
| `/atlas_loc/lidar/compact` | `sensor_msgs/msg/PointCloud2` | Compact converter → NDT and ground filter | Compact scan used by localization and perception. |
| `/atlas_loc/obstacles/pointcloud` | `sensor_msgs/msg/PointCloud2` | Ground filter → cluster detector and current guard | Non-ground cloud; candidate Nav2 costmap observation source, subject to height, clearing, and frame validation. |
| `/atlas_loc/obstacles/detected_objects` | `autoware_perception_msgs/msg/DetectedObjects` | Euclidean cluster detector → static-object adapter | Detection and cluster membership data. |
| `/atlas_loc/obstacles/predicted_objects` | `autoware_perception_msgs/msg/PredictedObjects` | Static-object adapter → Autoware obstacle planner and combined safety guard | Acquisition stamp is inherited from detection; never refresh it to hide processing delay. Nav2 costmaps do not consume this type directly without an adapter/plugin. |
| `/atlas_loc/ndt/ndt_pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | NDT → EKF and localization watchdog | Accepted map-frame measurement; source timestamp retained. |
| `/atlas_loc/ekf/ekf_odom` | `nav_msgs/msg/Odometry` | EKF → Autoware planner/controller/gate and ego marker | Estimated pose parent `map`, child `base_link`; nominal 50 Hz in the existing localization contract. This is the only operational estimated pose source. |
| `/atlas_loc/route_path` | `nav_msgs/msg/Path` | Route adapter → scene | Lanelet-derived route in `map`; transient-local. |
| `/atlas_loc/raw_trajectory` | `autoware_planning_msgs/msg/Trajectory` | Scene → obstacle planner | Scene's route-following reference. |
| `/atlas_loc/obstacle_trajectory` | `autoware_planning_msgs/msg/Trajectory` | Obstacle planner → velocity smoother | Obstacle-processed route. |
| `/atlas_loc/trajectory` | `autoware_planning_msgs/msg/Trajectory` | Velocity smoother → controller, base watchdog, RViz | Active smoothed trajectory. The base watchdog currently requires fresh trajectory input. |
| `/atlas_loc/control_cmd` | `autoware_control_msgs/msg/Control` | Autoware controller → vehicle command gate and base watchdog | Current Autoware-mode automatic command. |
| `/atlas_loc/gated/control_cmd` | `autoware_control_msgs/msg/Control` | Vehicle command gate → SimplePlanningSimulator input `input/ackermann_control_command` | Only accepted path to simulator actuation. |
| `/atlas_loc/base_safety_status` | `std_msgs/msg/String` | Base localization watchdog → combined perception guard | JSON health/latch status. |
| `/atlas_loc/base_safety_heartbeat` | `autoware_adapi_v1_msgs/msg/ManualOperatorHeartbeat` | Base watchdog → combined perception guard | Fresh base authorization heartbeat. |
| `/atlas_loc/safety_status` | `std_msgs/msg/String` | Combined perception guard → operator/validator | Combined guard health and authorization state. |
| `/atlas_loc/gate/input/external_emergency_stop_heartbeat` | `autoware_adapi_v1_msgs/msg/ManualOperatorHeartbeat` | Combined guard → vehicle command gate | Final authorization heartbeat; withheld on stale/unsafe inputs. |

The map loader accepts the campus OSM, PCD, and projector files and publishes Lanelet2 map data under `/atlas_map_demo`; NDT uses the synthetic differential point-cloud-map service at `/pcd_loader_service`. The five-lap scenario uses separate generated OSM/PCD/projector assets under `src/atlas_full_stack_demo/config/five_lap_loop/`. The original campus PCD is not established as registered to the current synthetic NDT map. See [full-stack architecture](../handoff/ARCHITECTURE.md) and [full-stack README](../../src/atlas_full_stack_demo/README.md).

### Simulation time

The existing launch has no `/clock` publisher and uses system ROS time (`use_sim_time: false`, also explicit in the Nav2 parameter file). Simulator messages, scene scans, and EKF measurements use ROS clock timestamps; freshness monitors compare source age in ROS time and receipt age on a monotonic clock. The Nav2 mode must retain this shared clock setting. A future simulated `/clock` mode would require one clock authority and consistent `use_sim_time` on every node; it is not part of this contract.

## 3. Existing Nav2 prototype interface

The separate [Nav2 launch](../../src/atlas_nav2_demo/launch/nav2_skid_steer.launch.py), [parameters](../../src/atlas_nav2_demo/config/nav2_params.yaml), [simulation configuration](../../src/atlas_nav2_demo/config/simulation.yaml), and [README](README.md) establish these interfaces:

| Interface | Type | Owner / use |
|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | Nav2 map server; transient-local static occupancy map loaded from YAML + PGM. |
| `/atlas_nav2/scan` | `sensor_msgs/msg/LaserScan` | Standalone ray-cast sensor → local costmap obstacle layer; frame `laser`. |
| `/atlas_nav2/odom` | `nav_msgs/msg/Odometry` | Standalone ideal simulator → BT navigator speed/goal checks and local controller odometry. |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | BT navigator action server; goal pose in `map`. |
| `/atlas_nav2/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 controller and behavior output in Humble fixture. |
| `/atlas_nav2/cmd_vel_safe` | `geometry_msgs/msg/Twist` | Standalone fixture's default-disabled gate output. |
| `/atlas_nav2/wheel_speed_targets` | `sensor_msgs/msg/JointState` | Fixture mixer → skid-steer simulator. |
| `/atlas_nav2/wheel_states` | `sensor_msgs/msg/JointState` | Ideal encoder feedback from fixture simulator. |
| `/atlas_nav2/safety_status` | `std_msgs/msg/String` | Fixture gate diagnostics. |

The launch activates map server, NavFn planner, controller server with DWB, behavior server, BT navigator, and lifecycle manager. Global costmap uses `map`; local costmap and behavior server use `odom`; both use `base_link`. Local obstacle input is LaserScan. The fixture controller and behavior commands are remapped to `/atlas_nav2/cmd_vel_nav`. The standalone gate clamps commands to configured limits, starts disabled, times out stale commands after 0.25 s, and emits zero while disabled or stopped. It is a test fixture, not a replacement for the Atlas Autoware command gate.

The fixture simulator owns `map → odom` (static identity), `odom → base_link` (dynamic), and `base_link → laser` (static). It has ideal no-slip skid-steer kinematics. The Autoware simulator is a different, ideal steering/acceleration geared model with provisional 1.20 m wheelbase and does not model the PRD's physical skid-steer drive. The model choice limits the meaning of any Nav2-on-Autoware-simulator result.

Current source package dependencies and installation are in [atlas_nav2_demo/package.xml](../../src/atlas_nav2_demo/package.xml) and [CMakeLists.txt](../../src/atlas_nav2_demo/CMakeLists.txt). The repository README documents Humble packages `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-nav2-simple-commander`, and `python3-pytest`; Simple Commander is optional for the current validator, which uses `NavigateToPose` directly. The manifest declares the specific Nav2 lifecycle, map, planner, controller, BT, DWB, NavFn and ROS message packages, but should explicitly declare `nav2_costmap_2d` because the configured costmap loads its plugins. During this audit, the sourced development environment did not resolve Nav2 runtime packages; the existing stable `.nav2_cache` contains the Atlas fixture package only. Phase 1 must re-establish dependencies without changing the stable overlay.

The fixture's tests and acceptance workflow are documented in [Nav2 validation](VALIDATION.md). Existing ARM64 evidence covers open goal motion, an avoidable obstacle detour, and a blocked-route safe failure. It validates the isolated idealized fixture only.

## 4. Proposed Nav2-mode contract

### Mode ownership

A future new launch file should select either Autoware mode or Nav2 mode; it must not include both controllers. Autoware mode remains byte-for-byte/configuration unchanged. Nav2 mode should start one Nav2 planner/controller, one active plant, one localization publisher, one set of safety monitors, and the existing vehicle command gate. Fail closed if both controller modes are requested or if command publisher counts are not exactly one.

### Localization and TF

The initial shared-mode candidate reuses the existing EKF without adding a transform broadcaster:

- EKF is the sole owner of estimated dynamic `map → base_link` on `/tf` and `/atlas_loc/ekf/ekf_odom`.
- Scene remains the sole owner of static `base_link → synthetic_lidar` and `base_link → imu_link` on `/tf_static`.
- The simulator may publish only its truth transform `map → hidden_truth_base`; Nav2, costmaps, and command conversion must never consume that truth frame or truth odometry.
- Nav2 uses `map` as `global_frame`, `base_link` as `robot_base_frame`, and `/atlas_loc/ekf/ekf_odom` as its odometry topic. To avoid inventing an `odom` transform, the first experiment configures both global and rolling local costmaps in `map`. This is a non-default Nav2 arrangement and must pass transform, rolling-costmap, localization-correction, and controller tests before use.
- No Nav2 fixture simulator or TF broadcaster is launched in this shared mode. In particular, do not publish an identity `map → odom`, a second `odom → base_link`, or a second `map → base_link`.

Candidate TF topology:

    map ──[EKF, sole estimated dynamic owner]──> base_link
                                                   ├──[scene static]──> synthetic_lidar
                                                   └──[scene static]──> imu_link
    map ──[simulator truth only]───────────────> hidden_truth_base

The prototype's standard `map → odom → base_link` tree is a separate-mode tree and is not layered onto this estimated tree. If Nav2's selected plugins cannot operate with the proposed map-frame local costmap, stop before motion and review a proper local-odometry plus map-correction design. Do not alias frames or republish EKF pose as a second transform to satisfy a default configuration.

All sensor and Nav2 messages must keep source acquisition timestamps. Sensor transforms use the same ROS time domain; transform lookups must be exact or use the existing bounded tolerance. The `/atlas_loc/ekf/ekf_odom` output frame and child frame must be asserted at startup and continuously monitored.

### Map and costmap

The current Nav2 8 m × 6 m PGM map has YAML origin `[-2, -3, 0]`; it cannot cover the approximately 71 m closed loop and terminal extension, whose synthetic lanelet coordinates span roughly 30 m in each axis. It can remain the standalone fixture map, but is not the Nav2 map for the five-lap route.

Recommended integrated map: generate a Nav2 OccupancyGrid YAML/PGM from the separate five-lap synthetic Lanelet2 road polygons, preserving their `map` coordinates and projector. Mark drivable lane area free, outside-of-map/non-drivable area occupied or unknown according to an explicit planner policy, and add obstacles only from the scenario definition or measured sensor data. Do not rasterize the NDT PCD as if it were an occupancy map: its road surfaces and NDT landmarks are localization geometry, not a semantic obstacle map. Do not alter the original campus assets. A campus-derived grid is a later option after projection, road semantics, asset alignment, and origin/resolution are validated.

Nav2 global costmap consumes `/map` (`nav_msgs/msg/OccupancyGrid`) from its mode's map server. For local observations, first evaluate `/atlas_loc/obstacles/pointcloud` (`sensor_msgs/msg/PointCloud2`) from the existing ground filter with a Nav2 obstacle observation source, sensor frame `synthetic_lidar`, bounded z filtering, acquisition-time checks, and verified marking/clearing behavior. If the Humble plugin/config cannot safely consume this cloud, use a tested PointCloud2-to-LaserScan adapter with explicit height/range limits. The existing `/atlas_loc/obstacles/predicted_objects` is not a Nav2 costmap input; do not silently discard its safety role or claim object-level integration.

### Command and safety

Nav2 controller Twist → Nav2-to-Autoware command adapter → `/atlas_loc/control_cmd` (`autoware_control_msgs/msg/Control`) → existing vehicle command gate automatic input → `/atlas_loc/gated/control_cmd` (`autoware_control_msgs/msg/Control`) → simulator `input/ackermann_control_command`.

Only the adapter publishes `/atlas_loc/control_cmd` in Nav2 mode; the Autoware controller is not launched. Only the existing gate publishes the simulator input. The adapter never publishes directly to the simulator or to the standalone Nav2 gate.

The adapter must:

1. Require finite `Twist.linear.x` and `Twist.angular.z`; reject NaN/Inf and unsupported nonzero lateral/y-axis commands.
2. Use an explicit, parameterized Ackermann conversion consistent with the simulator's configured vehicle model. For forward speed `v`, yaw rate `ω`, and validated wheelbase `L`, desired steering is `atan(L·ω/v)` when `|v|` is above a documented low-speed threshold. Clamp to simulator/vehicle steering limits and clamp longitudinal speed/acceleration to the existing vehicle limits. Stamp each output with current ROS time; track command receipt freshness locally with a monotonic clock.
3. Handle `|v|≈0` and nonzero `ω` explicitly. The Ackermann simulator cannot rotate in place. Such a command must cause a safe zero/rejected-command state and diagnostic; controller/configuration must not depend on spin-in-place. If skid-steer behavior is a product requirement, use the skid-steer plant rather than pretending the Ackermann mapping is equivalent.
4. On Nav2 command timeout, request a zero command through the existing gate once, then stop refreshing normal command output and allow the gate/watchdogs' own stale handling to act. Never synthesize authorization heartbeats or continuously fresh commands to conceal a lost Nav2 controller.
5. Remain inactive until Nav2 lifecycle nodes are active, localization/perception health is fresh, safety monitors are armed, and the existing explicit operator engagement authorization is granted. The gate remains authoritative and applies its configured limits/emergency behavior.

Safety-monitor compatibility is a launch blocker to resolve before motion. The base watchdog currently requires fresh `autoware_planning_msgs/msg/Trajectory` and `autoware_control_msgs/msg/Control` in addition to odometry, steering, acceleration, and NDT. The combined guard requires raw/converted clouds, ground cloud, predicted objects, base status, and base heartbeat. Keep the current 0.300 s freshness rules. In Nav2 mode, either feed the watchdog an actual Nav2 controller plan through a validated typed adapter with truthful frame/stamp/path semantics, or implement a reviewed mode-specific monitor that checks the actual Nav2 plan/action and command stream at the same freshness limit. A periodic fabricated trajectory or heartbeat is prohibited. Keep the existing perception outputs active until a separately reviewed Nav2-specific perception safety contract exists.

The default-disabled `/safety_gate` in `atlas_nav2_demo` remains only for its independent fixture. Reusing that gate in the shared Atlas mode would bypass the Autoware gate and watchdog authorization and is forbidden.

## 5. Adapters and launch files expected later

No files are created by this design. Likely implementation units, subject to the phase gates below:

- A new exclusive Nav2 mode launch, e.g. `atlas_nav2_mode.launch.py`, composed from reusable Atlas simulation/localization/perception/map actions while omitting Autoware route planner, velocity smoother, and controller.
- A Nav2 map generator/converter that creates an occupancy map from the synthetic five-lap Lanelet2 polygons in the same `map` coordinates; keep source and generated map assets separate from the original campus maps.
- A costmap observation configuration for the existing filtered PointCloud2, or a tested scan-projection node if required.
- A `Twist` → `autoware_control_msgs/Control` adapter with pure unit-tested kinematics, rate/finite/range checks, stale-command fail-safe, diagnostics, and no TF publisher.
- A truthful Nav2-plan/safety interface satisfying the existing monitor semantics without weakening freshness thresholds or synthesizing health.
- A mode preflight that checks exclusive controller ownership, publisher counts, map/frames, required topics, lifecycle state, fresh sensors, and monitor authorization before explicit engagement.

Do not modify `full_stack.launch.py` or the current Autoware route/controller launch to introduce Nav2. The existing working launch remains the fallback.

## 6. Staged implementation and acceptance

### Phase 1 — Nav2 dependencies and standalone prototype verification

Use a separate development-only install/build prefix; do not source or overwrite the stable demo overlay. Restore the documented Humble Nav2 dependencies and build only `atlas_nav2_demo` after dependency presence is confirmed.

Pass criteria:

- Every executable/plugin named by `nav2_skid_steer.launch.py` resolves from the intended Humble install.
- `atlas_nav2_demo` tests pass.
- The existing open, avoidable, and blocked validator scenarios reproduce their documented results in fresh evidence directories.
- Lifecycle state, map/costmaps, TF, disabled gate, command freshness, and safe blocked-route behavior are captured. A previous run or compile alone does not count as a new pass.

### Phase 2 — Shared simulation and localization, motion disabled

Create a new Nav2-mode launch that shares the existing synthetic sensor, NDT/EKF, and perception graph but does not launch either motion controller or start engagement. Disable the Nav2 fixture simulator/gate. Use only EKF odometry and the EKF transform for the estimated state.

Pass criteria:

- No vehicle command reaches the simulator; standstill is verified from truth only as an evaluator signal.
- Exactly one estimated `map → base_link` TF authority exists (EKF).
- No `odom → base_link` or truth pose is consumed by Nav2.
- EKF and all safety monitor inputs remain fresh for a bounded soak; monitors remain unlatched.

### Phase 3 — Map, TF, and costmap validation

Generate the five-lap occupancy grid from the same synthetic lane polygons; start map server and Nav2 planner/costmaps with the candidate `map`-frame localization contract and actual filtered sensor observation.

Pass criteria:

- Occupancy map origin, resolution, bounds, and lane free/blocked cells match the five-lap map geometry.
- Nav2 can transform `map` to `base_link` and `synthetic_lidar` at sensor stamps without duplicate TF authorities.
- Costmap observes a known obstacle and clears free space correctly after sensor motion/absence.
- A global plan follows the drivable loop and does not cut across occupied road boundaries.
- Localization corrections do not leave a stale or inconsistent costmap.

### Phase 4 — Command adapter and safety integration

Unit-test the Twist-to-Control mapping and fail-safe behavior. Test Nav2-mode launch exclusivity and gate inputs with motion disabled, then with a bounded stationary/low-speed command only after explicit authorization.

Pass criteria:

- One publisher on `/atlas_loc/control_cmd`, zero Autoware controller instances in Nav2 mode, one gate output publisher.
- Non-finite, out-of-range, lateral, rotate-in-place, and stale command cases are rejected or produce safe zero through the gate.
- No movement before operator engagement; stale input causes stop; both watchdogs preserve existing 0.300 s thresholds and remain responsive.
- Fault tests show authorization is withheld and no automatic rearm occurs.

### Phase 5 — Short-distance navigation goal

Run a short clear goal with a fresh evidence directory, then repeat it. Compare EKF pose, planned path, commands, gate output, and actual simulated displacement.

Pass criteria:

- Nav2 action succeeds within goal tolerance; actual EKF and truth paths show motion to the goal without truth feeding control.
- Tracking and localization metrics are reported; command and sensor freshness remain valid.
- No watchdog/perception latch, duplicate control publisher, or transform conflict.
- The vehicle stops and holds at goal with explicit completion evidence.

### Phase 6 — Obstacle and blocked-route tests

Use a costmap obstacle represented in both map and live sensor data; then test a dynamically sensed avoidable obstacle and a blocked route.

Pass criteria:

- Avoidable case detours with measured footprint clearance and reaches the goal.
- Blocked case aborts or reports no valid path, and the gate outputs zero; no collision/integration through occupied cells.
- Stale/missing observation causes the existing monitor path to withhold authorization without changing thresholds.
- Preserve all failed trial evidence. Do not start five-lap testing until repeated short clear and obstacle runs pass.

## 7. Risks and mitigations

| Risk | Mitigation / acceptance evidence |
|---|---|
| Two controllers publish commands | Separate launch modes; Nav2 launch excludes Autoware controller and vice versa; assert one publisher before arming. |
| TF conflict or inconsistent frames | EKF owns estimated map→base_link; scene owns sensor static transforms; do not launch Nav2 fixture TF in shared mode; inspect TF graph before enabling commands. |
| Missing odom frame in EKF output | First test Nav2 with map-frame local costmap and EKF odometry topic. If unsupported, stop and design a real local odometry/map correction architecture; do not fabricate identity or duplicate transforms. |
| Occupancy map differs from Lanelet/PCD | Rasterize only the synthetic Lanelet road polygons in their exact map coordinates; validate origins/bounds/free-space; keep PCD as localization geometry. |
| Point cloud height/ground geometry pollutes costmap | Configure bounded height and clearing behavior; compare costmap obstacle cells to known synthetic obstacles; require obstacle/free-space evidence. |
| Nav2 Twist not physically equivalent to four-wheel skid-steer | Treat Autoware's ideal steering simulator as a software-only bridge; no hardware claims. Human review must choose Ackermann-like simulation vs target skid-steer plant before performance claims. |
| Safety watchdog expects Autoware trajectory and control | Provide a truthful mode-specific plan/command health input at unchanged 300 ms freshness, or stop; never send synthetic refresh messages. |
| Localization or sensor freshness failures | Keep NDT/EKF, perception, and existing watchdogs; preserve source stamps and receipt diagnostics; no threshold relaxation. |
| Resource load from two stacks | Never run both planners/controllers concurrently; measure CPU/memory and callback gaps serially on the ARM64 VM. |
| PRD authority uncertain | Compare provisional requirements to official ROAM PDR before freezing a production architecture. |

## 8. Human decisions before motion integration

1. Confirm from the official PDR whether Nav2 must own global/local planning and control, and whether Autoware planning/control may remain as a separate operational mode.
2. Confirm target drivetrain semantics: the PRD calls for four-wheel skid-steer, while the current Autoware full-stack simulator is Ackermann-like. Decide which plant is authoritative for Nav2 integration evidence.
3. Approve the initial no-odom-frame Nav2 experiment (map-frame local costmap using EKF `map → base_link`) or request a true local odometry / map-correction architecture.
4. Approve the safety-monitor Nav2 plan interface that preserves the existing freshness limit and fail-closed authorization.
5. Select whether Nav2 obstacle costmaps consume ground-filtered PointCloud2 or a validated LaserScan projection.

## 9. First implementation phase

Begin with Phase 1 only: restore/verify Nav2 Humble dependencies in a development-only environment, build the existing standalone package in a new external prefix, and reproduce its documented open, avoidable, and blocked tests. This establishes a known-good Nav2 baseline before creating any shared launch or command bridge. The currently checked development environment did not resolve Nav2 runtime packages, so dependency provenance is a concrete prerequisite rather than an assumed condition.

After Phase 1, implement Phase 2 with motion disabled. Do not begin command conversion until the frame, map, costmap, and safety-monitor contracts above pass their gates.
