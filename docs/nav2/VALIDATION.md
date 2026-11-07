# Nav2 fixture validation

## Drivetrain status correction

Atlas is confirmed **Ackermann steering**. The skid-steer launch and evidence in
this document are retained as a historical isolated fixture only; they do not
verify Atlas's intended drivetrain. The intended fixture is now
`nav2_ackermann.launch.py`: it uses a bicycle model, bounded steering/speed,
and `autoware_control_msgs/Control` as the future vehicle-command boundary.
It is `IMPLEMENTED_NOT_VALIDATED` until its own runtime cases pass.

### Ackermann runtime evidence: 2026-11-07

The Ackermann fixture is built in
`~/roam/diagnostic_overlays/atlas_ackermann_v1/install`; that overlay was
explicitly sourced after Humble.  A bounded headless straight-path run in ROS
domain 202 accepted and completed a `NavigateToPose` goal without a runtime
Spin request:

`validation_evidence/ackermann/open_verified_20261107T070300Z/summary.json`

The action returned `SUCCEEDED`; the capture includes 700 odometry samples,
693 `Control` command samples, 0.50 rad maximum steering (the configured
bound), 0.55 m/s maximum command speed, and exactly one publisher each for
`/atlas_ackermann/odom` and `/tf` (`ackermann_simulator`).  This is evidence
for graph bring-up, bounded bicycle motion, command conversion, gate enable,
and TF ownership only.

It is deliberately **not** Ackermann-navigation acceptance: the obstacle
sensor/costmap fixture and the curved/minimum-radius/blocked scenarios have
not yet been implemented and measured.  Earlier failed records
`ackermann/open_20261107T064600Z` and `open_straight_20261107T065300Z` are
retained; they exposed a Humble parameter-type mismatch, planner-ID mismatch,
and an incompatible stock spin-recovery tree.  The current launch fixes those
configuration defects with a bounded no-spin `NavigateToPose` tree.

### Ackermann obstacle and blocked-route evidence: 2026-11-07

`validation_evidence/ackermann/obstacle_graph_20261107T071000Z` shows the
Ackermann scan publisher, local `ObstacleLayer`, and active Nav2 lifecycle.
The scan is ray-cast from the simulator pose; obstacle geometry is not passed
to Nav2 except through `LaserScan`.

The blocked scenario is a safety pass, not navigation completion:
`validation_evidence/ackermann/blocked_20261107T072300Z` returned `ABORTED`
because Smac found no route, and captured an all-zero safe command. The custom
NavigateToPose tree did not invoke a Spin recovery.

`validation_evidence/ackermann/safety_gate_20261107T072500Z` passed isolated
live gate injection for missing/stale localization, invalid timestamps,
missing/stale commands, disabled operation, emergency stop, and emergency clear
without automatic re-enable. It verifies immediate zero command behavior; the
bicycle simulator's bounded deceleration remains a model, not physical stopping
distance evidence.

Avoidable-obstacle attempts are retained as failures, including
`avoidable_probe_20261107T071100Z`, `avoidable_startwest_20261107T071500Z`,
and `avoidable_reeds_20261107T072000Z`. They establish that controller
collision projection and progress checks fail safely, but they do not establish
planned avoidance. In this 4 GB QEMU VM, the latter trace also recorded repeated
BT/controller deadline misses. Ackermann Nav2 therefore remains
`IMPLEMENTED_NOT_VALIDATED`.

`atlas_nav2_demo` is an isolated simulation fixture. It has its own `map`,
`odom`, `base_link`, laser scan, safety gate, wheel mixer, and ideal encoders;
it does not connect to the Autoware command gate or physical vehicle topics.

The only `odom -> base_link` publisher is `skid_steer_simulator`. It publishes
ideal simulated encoder states. `map -> odom` is a static identity transform for
this fixture only. Production localization must replace that selection through a
separate reviewed launch mode, not by running duplicate TF publishers.

The Nav2 controller publishes `/atlas_nav2/cmd_vel_nav`. `safety_gate` alone
may publish `/atlas_nav2/cmd_vel_safe`. It begins disabled; it requires fresh
command receipt and fresh, valid-source-stamped `/atlas_nav2/odom`; it clamps
commands and publishes zero for disabled, emergency-stop, stale command, stale
localization, invalid localization, or shutdown conditions. `wheel_mixer` then
converts the safe Twist to four targets and the simulator applies a separate
target timeout.

## Runtime prerequisites and command

Nav2 Humble packages must be installed before attempting this optional fixture.
Do not run it alongside the Autoware full-stack control graph.

### Dependency check: 2026-11-07

This VM is Ubuntu Jammy ARM64 (`arm64`/`aarch64`). Official ROS 2 Humble
packages are available from the configured `packages.ros.org` Jammy ARM64
repository: `ros-humble-navigation2` and `ros-humble-nav2-bringup`, including
the map server, planner, controller, DWB, behavior server, BT navigator, and
lifecycle manager used by this launch. They were not installed at inspection.

The simulated transaction contains 77 new packages and approximately 159.5 MiB
of installed package payload (sum of APT `Installed-Size`; downloaded archives
and package-manager overhead are additional). The VM had 13 GiB free, so disk
capacity is adequate. Installation requires administrator authentication; do
not substitute a source build or alter the protected Autoware checkout.

```bash
sudo apt-get update
sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup
```

After installation, use a fresh domain and the development overlay. No RViz is
needed:

```bash
source /opt/ros/humble/setup.bash
source ~/roam/diagnostic_overlays/atlas_prd_v02_sprint_v1/install/setup.bash
export ROS_DOMAIN_ID=191
ros2 run atlas_nav2_demo validate_closed_loop.py \
  --scenario open --output ~/roam/validation_evidence/nav2/open_$(date -u +%Y%m%dT%H%M%SZ)
```

The validator checks lifecycle activation, map/costmap/TF availability, goal
acceptance, command generation, gated wheel targets, encoder-derived odometry,
post-goal zero output, disabled-gate zero output, and obstacle scenarios. A
passing run is simulation-only evidence; it is not physical vehicle validation.

Run one bounded trial each for `open`, `avoidable`, and `blocked`, retaining all
summaries and launch logs. The safety gate's dependency-free tests cover stale
and missing localization, stale commands, disabled operation, emergency stop,
and saturation; runtime testing must additionally observe those conditions in
the installed graph before Nav2 may be promoted beyond
`IMPLEMENTED_NOT_VALIDATED`.

## Runtime evidence: 2026-11-07

Official Nav2 Humble binaries were installed and resolved from `/opt/ros/humble`.
Atlas packages resolved from
`/home/marian/roam/diagnostic_overlays/atlas_prd_v02_sprint_v1/install`.
All runs were headless and isolated from Autoware.

| Scenario | Result | Evidence | Key result |
|---|---|---|---|
| Open | PASS | `validation_evidence/nav2/open_20261107T062000Z` | Lifecycle, map/costmaps, TF, goal completion, wheel commands, zero-after-goal and disabled-gate stop passed; final goal error 0.179 m. |
| Avoidable | PASS | `validation_evidence/nav2/avoidable_20261107T062100Z` | Goal completed with obstacle lidar/costmap evidence, lateral detour, and 0.271 m minimum geometric clearance. This is planned navigation, not a watchdog stop. |
| Blocked | PASS | `validation_evidence/nav2/blocked_20261107T062200Z` | Navigation failed/rejected as expected (`action_status: 6`), obstacle was in lidar/costmap, safe output stopped, and no collision occurred. |
| Gate safety | PASS | `validation_evidence/nav2/safety_gate_20261107T062600Z` | Missing/stale localization, invalid timestamps, stale commands, emergency stop, and non-rearm after emergency clear all produced zero output. |

The earlier failed safety-gate harness records are retained as evidence. They
identified only validator sampling/startup races; the corrected bounded run is
the passing record above.

`validation_evidence/nav2/graph_ownership_20261107T062700Z` captured one live
publisher each for `/atlas_nav2/odom`, `/tf`, and `/tf_static`: all were
`skid_steer_simulator`. Thus the fixture had no competing odometry or TF
authority. This identity `map -> odom` arrangement remains a simulation-only
assumption and must not be reused for deployment.
