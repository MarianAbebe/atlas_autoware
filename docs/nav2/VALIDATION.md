# Nav2 fixture validation

> **Current status note (2026-11-11):** the earlier sections below record
> historical RPP/MPPI development evidence. The active custom Nav2 Humble
> Ackermann controller plugin and its bounded campaign are documented in the
> [current integration status](NAV2_INTEGRATION_STATUS.md). Do not use old
> RPP/MPPI aborts as evidence about the new plugin, and do not treat them as
> superseded safety failures: their retained traces explain why a curvature-
> constrained controller was implemented.

## Final acceptance refresh: 2026-11-11

The final-code regression and fault-injection records are under
`~/roam/validation_evidence/ackermann/final_acceptance_20261111/`. Exact
scenario summaries and the completion decision are in
[`NAV2_INTEGRATION_STATUS.md`](NAV2_INTEGRATION_STATUS.md).

After adding controller-side LaserScan freshness and synchronized dynamics
diagnostics, a fresh regression passed straight 5/5 and avoidable obstacle
5/5; blocked route aborted safely 1/1. A five-run curved campaign had four
successes and one safe abort during a multi-second timer interruption. A
subsequent independent curved campaign passed 5/5. The failed trace is
retained. Avoidable-route clearance was approximately 0.64 m after the
configured 0.32 m circular footprint. Simulator integration steps measured
+0.80 / -1.20 m/s² at configured limits. These are simulator-model
measurements, not hardware performance.

Full-graph fault injection exercised odometry loss, scan loss, controller
command loss, emergency stop and explicit rearm, navigation cancellation, and
non-finite command input. All six checks passed for their modeled stopping
envelopes. A subsequent 20-trial cancellation campaign recorded gate zero
publication at 69.1–106.4 ms (median 79.4 ms), safe-zero observation at
69.7–106.8 ms, adapter zero at 70.2–107.2 ms, and simulator zero-target
application at 76.0–110.6 ms. All 20 were below 250 ms. Simulated standstill
was 176.0–230.5 ms; maximum post-fault travel was 0.0232 m, below each
trial's conservative modeled envelope. The old 363.8 ms receipt-only result
is retained; its original trace cannot distinguish publisher delay from
recorder delivery delay.

The same investigation found a separate source-timestamped curved-run
interruption: the gate timer had a 731 ms interarrival gap and the simulator a
758 ms tick gap. The gate resumed with stale command/localization and emitted
zero. That run safely aborted. The exact scheduler cause is unresolved, and
the common OS/VM dependency means the cancellation campaign does not prove
timing behavior during a system-wide stall. Current milestone status is
**navigation validated; timing acceptance open**. See
[`NAV2_INTEGRATION_STATUS.md`](NAV2_INTEGRATION_STATUS.md) and
`~/roam/validation_evidence/ackermann/final_acceptance_20261111/timing_investigation/`
for monotonic-time records and resource samples.

The runtime recorder now consumes `/atlas_ackermann/simulator_dynamics`, which
publishes each simulator integration step with a shared simulation timestamp,
the exact integration interval, before/after speed, acceleration, steering
rate, pose, and collision clamp. Use this channel for simulator-limit
verification; do not infer exact integration acceleration by differencing
asynchronous odometry headers. The fault runner retains source stamps and
monotonic receipt times separately and reports graph interarrival gaps around
the first safe-zero output.

The controller refuses motion if `/atlas_ackermann/scan` is missing, source
timestamp-invalid/stale, or older than its configured 0.30 s monotonic receipt
limit. Costmap collision checking remains enabled. The fault harness pauses
odometry and scan through simulator test services; it does not route simulator
ground-truth obstacles into Nav2.

## Earlier custom-plugin campaign snapshot: 2026-11-11

The following campaign was completed before the final acceptance refresh above.
Its navigation results remain valid historical records; its statements that
full-graph fault injection was unverified and that acceleration measurement
was unresolved were superseded by the synchronized diagnostics and fault
campaign in the final acceptance refresh.

The current Humble `nav2_core::Controller` plugin is exercised by
`scripts/run_ackermann_acceptance.py` through the actual controller server,
Smac planner, current LaserScan-fed costmap, safety gate, adapter, bicycle
simulator, odometry, and TF. See [`NAV2_INTEGRATION_STATUS.md`](NAV2_INTEGRATION_STATUS.md)
for the controller equations, assumptions, and remaining acceptance items.

The final configuration passed five consecutive straight, five curved, and
five avoidable-obstacle action runs. Each successful run waited for a stopped
pose inside the recorder's configured goal tolerance. All adapter status
samples were feasible, the safety gate remained enabled with fresh localization
and command and no E-stop, and the avoidable route followed the recorded Nav2
path around scan/costmap-observed geometry. Measured avoidable-route clearance
to the rectangular obstacle, after subtracting the simulated 0.32 m circular
radius, was 0.638–0.643 m; maximum cross-track distance to the published path
was 0.035–0.040 m. The blocked scenario returned `ABORTED` while remaining
within 0.02 m of its initial pose and publishing only zero safe commands.

| Scenario | Attempts | Passes | Position / constraint evidence |
|---|---:|---:|---|
| Straight | 5 | 5 | XY goal error 0.168–0.171 m, stopped; speed max 0.20 m/s; steering approximately 0 rad. |
| Curved | 5 | 5 | XY error 0.169–0.174 m, yaw error 0.074–0.076 rad; max steering 0.49985–0.49996 rad; max measured steering rate 0.6000001 rad/s. |
| Avoidable obstacle | 5 | 5 | XY error 0.170–0.173 m, yaw error 0.0496–0.0503 rad; clearance 0.638–0.643 m; max cross-track 0.035–0.040 m; all commands adapter-feasible. |
| Blocked route | 1 | 1 safe abort | Action `ABORTED`; 62 odometry samples, no displacement above 0.02 m; 67 zero safe commands; no E-stop. |

Evidence is in `validation_evidence/ackermann/nav2_plugin_20261111/`:
`straight_repeat5_final/`, `curved_repeat5_final/`,
`avoidable_repeat5_final/`, `blocked_final_after_readiness/`, and
`safety_gate_regression/`. The first blocked trial in `blocked_final/` is
retained as a recorder startup/discovery failure because it had no odometry
samples. The recorder was changed to await fresh odometry and scan before goal
submission; the follow-up blocked run captured odometry and passed the safe
abort assertion.

At this snapshot, safety verification consisted of the isolated eight-check
gate suite; the full-graph results are in the final acceptance section above.
Timestamp-differenced odometry in earlier runs ranged to +0.877/-1.201 m/s²
because message-header and receipt timing was not aligned with integration
steps. The later simulator-dynamics stream resolves the model-level measurement
question, not physical stopping or acceleration validation.

## Drivetrain status correction

Atlas is confirmed **Ackermann steering**. The skid-steer launch and evidence in
this document are retained as a historical isolated fixture only; they do not
verify Atlas's intended drivetrain. The intended fixture is now
`nav2_ackermann_controller.launch.py` (the launch used by the final acceptance
campaign); the 2026-11-07 fixture referenced below was
`nav2_ackermann.launch.py`. Both use a bicycle model, bounded steering/speed,
and `autoware_control_msgs/Control` as the future vehicle-command boundary.
At the time of the 2026-11-07 evidence below, this fixture was not yet
validated; current status is superseded by the final acceptance refresh above.

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

For the Ackermann launch, `ackermann_simulator` is the sole
`odom -> base_link` and `/atlas_ackermann/odom` publisher; it publishes ideal
simulated encoder states. `map -> odom` is a static identity transform for this
fixture only. The skid-steer launch retains its separate
`skid_steer_simulator` publisher. Production localization must replace that
selection through a separate reviewed launch mode, not by running duplicate TF
publishers.

In the historical skid-steer launch, the Nav2 controller publishes
`/atlas_nav2/cmd_vel_nav`; `safety_gate` alone may publish
`/atlas_nav2/cmd_vel_safe`. The Ackermann launch uses the corresponding
`/atlas_ackermann/` topics and the same gate contract. Both gates begin
disabled, require fresh command receipt and fresh, valid-source-stamped odometry,
and publish zero for disabled, emergency-stop, stale command, stale
localization, invalid localization, or shutdown conditions. The skid-steer
`wheel_mixer` converts its safe Twist to four targets; Ackermann uses
`ackermann_adapter` to create bounded steering and velocity commands.

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
the development colcon overlay sourced for that validation run. The current
overlay path is selected through `ATLAS_NAV2_OVERLAY` in the native validation
guide.
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

## Avoidable-route diagnosis: 2026-11-08

Bounded runs reproduced the avoidable-route abort with the current development
overlay, no RViz, and isolated ROS domains (213–219). The goal was
`map (4.5, 0.0)` from the simulator start `(-1.0, 0.0)` with the existing
central rectangular obstacle. The action was accepted and a 37-pose Smac path
was published, but the vehicle remained at `(-1.0, 0.0)` and the action aborted
after the controller progress timeout. The original launch log contains
repeated `Rejected infeasible curvature request` messages followed by
`Failed to make progress`; this is not an obstacle-avoidance pass.

The captured path routes below the obstacle (`minimum y = -0.890 m`). At the
obstacle's lower edge (`y = -0.25 m`), the centerline-to-edge distance is about
0.640 m, leaving about 0.320 m after the configured 0.32 m circular footprint.
The path begins at the nearest 0.1 m map-cell center `(-1.05, -0.05)`, offset
from the exact simulated pose. The default RPP minimum lookahead is 0.35 m; at
near-zero initial speed it requests curvature beyond the configured steering
authority, which the adapter correctly rejects. Confidence is high that this
controller/adapter mismatch explains the initial no-motion/progress failure.
This is not isolated to the obstacle geometry: an obstacle-free goal from the
same initial pose also aborted with a measured request of `-1.390 1/m` / `-1.031
rad`. Aligning the open-route start with a 0.1 m map cell center reduced the
request but still exceeded the steering limit (`0.569 1/m`, `0.599 rad`). A
0.70 m minimum lookahead still requested `0.530 1/m`, `0.566 rad`. Thus the
current RPP configuration is incompatible with the configured steering bound
even on the open route; confidence is high in this diagnosis.

Diagnostic parameter probes were kept outside the source tree:

| Probe | Result | Evidence |
|---|---|---|
| Fixed lookahead 1.5 m | Still rejected infeasible curvature; progress abort | `validation_evidence/ackermann/diagnose_20261108T/lookahead_launch.log`, `lookahead_action.txt` |
| Fixed lookahead 2.0 m | No adapter curvature-rejection log; RPP collision projection aborted | `.../lookahead20_launch.log`, `lookahead20_action.txt` |
| Fixed lookahead 2.5 m | RPP collision projection aborted; run overlapped a previous graph during teardown and is inconclusive | `.../lookahead25_launch.log`, `lookahead25_action.txt` |
| Fixed lookahead 2.0 m, Smac `cost_penalty` 12 and 30 | Collision projection still aborted; path remained at `minimum y = -0.890 m` | `.../widepath_launch.log`, `.../clearance30_launch.log`, `clearance30_plan.yaml` |

An open-route run using the rebuilt adapter confirmed that rejected-command
logs are throttled and now report the actual requested curvature/steering.
That run still aborted safely; the diagnostic correction did not make the
controller request feasible. Evidence: `postfix_open_launch.log` and
`postfix_open_action.txt`. A map-cell-centered open-route probe with a 0.70 m
minimum lookahead also aborted; evidence is in `cellcenter_open_*` and
`centered07_open_*`.

This establishes a planner/controller feasibility mismatch. A synchronized
one-goal recorder was added at
`src/atlas_nav2_demo/scripts/record_ackermann_navigation.py`. It writes bounded
JSONL records for `/plan`, commands, adapter/gate status, odometry, LaserScan,
local/global costmaps and footprints, TF, action feedback/results, and relevant
`/rosout` entries. Each record carries ROS time, message header/source time
where available, monotonic receipt time, and callback duration. Output creation
is exclusive; the recorder has a deadline and disables the gate on completion
if it enabled the gate. Action feedback is sampled at no more than 10 Hz to
avoid serializing the behavior-tree feedback stream at its much higher rate;
sensor, command, odometry, safety, TF, costmap, warning, and terminal action
records are not decimated.

For the fixed 2.0 m RPP lookahead trace
`validation_evidence/ackermann/diagnose_20261108T/capture_rpp2m.jsonl`, the
goal was accepted at 2026-11-08 20:40:38.541646 UTC and a 37-pose plan
arrived at 20:40:38.551806 UTC. The first nonzero controller command was
received at 20:40:38.570249 UTC. Vehicle-to-nearest-plan distance exceeded
0.10 m by 20:40:38.871731 UTC and 0.30 m by 20:40:42.491565 UTC. This
tracking divergence preceded the collision warning; it does not show that
the planned path itself intersects the obstacle.

first RPP `detected collision ahead` warning was at ROS time
2026-11-08 20:40:42.768530 UTC (monotonic receipt 32359750709107 ns). Pose was
approximately `(1.055, -0.459) m`, yaw `-0.249 rad`, speed 0.55 m/s. The last
controller command before the warning was `v=0.55 m/s`, `w=0.0986 rad/s`,
`k=0.1793 1/m`, within the adapter's steering limit. The latest scan was
stamped 2026-11-08 20:40:42.670654 UTC, about 98 ms before the warning. Scan
receipt gaps peaked at 137 ms; odometry at 28 ms; TF at 33 ms. The gate was
enabled with fresh localization and command and no E-stop. This was not a
watchdog stop. The action aborted at 20:40:42.793229 UTC, about 25 ms after
the warning. This trace contains no controller-loop-missed warning before
the abort. The recorder's callback duration is its own serialization time,
not internal Nav2 plugin execution time, so it cannot establish per-plugin
callback latency.

Integrating that command for one second from the recorded pose gives a
projected center near `(1.593, -0.569) m`. Distance to the nearest obstacle
corner `(1.75, -0.25) m` is about 0.356 m, leaving approximately 0.035 m after
the simulator's 0.32 m circular footprint. The costmap resolution is 0.10 m
and inflation radius 0.65 m. The collision prediction is geometrically
plausible and conservatively valid; evidence does not support disabling or
weakening it. The exact first lethal costmap cell at decision time remains
unknown because local master/raw costmaps published at about 1 Hz, with the
nearest snapshot around 0.67 s before the warning. The scan itself was current.
Humble RPP's `max_allowed_time_to_collision_up_to_carrot` defaults to 1.0 s;
the controller limits projection by the selected carrot distance, and the
trial did not override this parameter.

The vehicle had already diverged from the global plan before this event: near
`x=1.05 m`, the path was around `y=-0.68…-0.74 m`, while odometry was near
`y=-0.46 m`. The planner path is not evidence of the controller's tracked
swept path. The sampled Smac plan's heading curvature reaches about 0.465
1/m (radius 2.15 m) in a local finite-difference estimate, slightly tighter
than its configured 2.20 m; reverse cusps make that estimate uncertain, so
this is a planner-segment validation warning rather than proof of infeasibility.

### Bounded controller comparison: 2026-11-09

All probes were headless, isolated, and retained as evidence under
`validation_evidence/ackermann/controller_eval_20261109/`. No trial changed
the adapter's 0.50 rad limit, collision checking, freshness behavior, or gate.

| Probe | Result | Evidence |
|---|---|---|
| RPP fixed 1.20 m lookahead | ABORTED without motion; 600/609 adapter statuses rejected; request reached 0.552 1/m / 0.585 rad. | `avoidable_rpp12_run1.jsonl` |
| RPP fixed 1.80 m lookahead | ABORTED without motion; 597/606 statuses rejected; request 0.494 1/m / 0.535 rad. | `avoidable_rpp18_run1.jsonl` |
| Humble MPPI Ackermann, min radius 2.20 m, reverse allowed | ABORTED; 275 adapter rejects; measured Twist curvature peaked 0.757 1/m. Vehicle reached about `(1.31, 0.20) m`; one 10 Hz controller-loop miss occurred. | `../mppi_probe_20261108/avoidable_mppi_footprint_domain228.jsonl` |
| Same MPPI trial with reverse disabled (`vx_min=0`) | ABORTED; curvature still reached 0.482 1/m / 0.524 rad; 609/1055 status records were rejected; vehicle stopped near `(1.21, 0.28) m`. | `avoidable_mppi_forwardonly_run1.jsonl` |

Humble MPPI provides an Ackermann sampled-motion constraint, but these runs
show the emitted Twist in this setup is not guaranteed to obey the adapter's
curvature bound, even with reverse disabled. Low-speed nonzero angular
commands are also rejected by the adapter's safe low-speed policy. The
evidence-only MPPI config needed a costmap polygon when its CostCritic used
footprint collision checks; its 16-sided, 0.322 m circumradius polygon
conservatively encloses the simulator's 0.32 m circle. The plugin initialized
with this footprint. MPPI is not promoted as a drop-in alternative.

RPP remains useful as a comparison fixture but is not currently validated for
this Ackermann vehicle: shorter tested lookaheads produce infeasible requests;
the feasible 2.0 m trial's predicted arc approaches the obstacle within the
costmap/collision margin and aborts safely. MPPI likewise has not completed
the goal and violates the end-to-end curvature contract. These failures are
not explained solely by VM contention: RPP's regular scan/odom/TF timing and
explicit adapter rejects explain the short-lookahead failures. MPPI did miss a
controller deadline once, which may contribute to repeatability but does not
explain the measured curvature violations.

The pre-trial memory snapshot showed 3.8 GiB RAM total, about 1.9 GiB
available, and 1.1 GiB swap used. CPU and peak RSS were not sampled
continuously during these captures, so this run set cannot establish or exclude
short CPU/memory pressure. Do not attribute the navigation abort solely to
QEMU scheduling based on the single MPPI deadline warning.

No controller/configuration combination in this comparison completed the
avoidable goal; acceptance remains unmet. The next engineering step is an
Ackermann-aware controller whose **published** command is bounded by
wheelbase, steering angle, steering rate, low-speed behavior, and the
independent swept-footprint collision check. Test the command/trajectory
contract deterministically before connecting it to Nav2. Preserve RPP and
MPPI configs for comparison. Do not clip adapter commands, raise steering
limits, shrink collision geometry, disable collision checks, or count a safe
abort as navigation success.

Reproduce a capture with the development overlay on a supported Fast DDS
domain, for example:

```bash
source /opt/ros/humble/setup.bash
source ~/roam/diagnostic_overlays/atlas_ackermann_v1/install/setup.bash
export ROS_DOMAIN_ID=230
ros2 launch atlas_nav2_demo nav2_ackermann.launch.py scenario:=avoidable
```

In a second sourced terminal on the same domain:

```bash
ros2 run atlas_nav2_demo record_ackermann_navigation.py \
  --output ~/roam/validation_evidence/ackermann/next_trial.jsonl \
  --goal-x 4.5 --goal-y 0.0 --timeout 40 --enable-gate
```

Use a new output path for each trial. The capture includes controller warnings,
action status, global path, scans/costmaps, TF, odometry, gated commands, and
adapter feasibility status on a common ROS and monotonic timeline.

### Code regression tests and build: 2026-11-08

`atlas_nav2_demo` built successfully in the isolated
`~/roam/diagnostic_overlays/atlas_ackermann_v1` overlay with
`CMAKE_BUILD_PARALLEL_LEVEL=1`. The complete pytest suite passed 31 tests,
including five new adapter-conversion cases. The CTest/colcon result contains
32 cases total (31 pytest cases plus the CTest wrapper), with zero errors or
failures. These are software regression results; they do not change the failed
avoidable-route runtime status.

### Regression retest: 2026-11-09

The package rebuilt in the isolated `atlas_ackermann_v1` overlay with
`CMAKE_BUILD_PARALLEL_LEVEL=1` and `MAKEFLAGS=-j1`. `colcon test` then reported
32 tests, 0 errors, 0 failures, and 0 skipped. Python syntax checks passed for
the recorder, adapter/helper, synthetic LaserScan publisher, and Ackermann
launch. The installed recorder `--help` check passed. The runtime controller
probes above all ended in ABORTED; regression tests do not convert them to
navigation passes.

### Isolated path-tracker prototype: 2026-11-11

The new ROS-independent `ackermann_path_controller.py` passed its deterministic
unit tests and a bounded fixed-step harness. The harness completed straight,
curved, and feasible obstacle-detour paths, stopped safely on an infeasible
curvature request, and stopped before a blocked obstacle. Evidence is in
`validation_evidence/ackermann/controller_prototype_20261111/summary.json`;
the initial harness fixture/stop-loop failures are retained in
`summary_initial_harness_failures.json`. The detour goal error was 0.0841 m,
minimum modeled obstacle clearance 0.1129 m, peak steering 0.4943 rad, and final
speed zero. The blocked-route stop retained only 0.00058 m clearance beyond
the assumed footprint and margin and is not a robust braking-distance result.

The package test suite reports 53 tests, 0 errors, 0 failures, and 0 skipped
after adding the prototype tests. This prototype is not connected to the Nav2
controller server and does not resolve the failed live Nav2 avoidable route.
The captured Smac path also remains inconclusive under discrete curvature
sampling: its maximum adjacent-pose estimate is 0.464910 1/m versus a
0.455252 1/m limit, while its eight-edge estimate is 0.449611 1/m. See
[`ACKERMANN_CONTROLLER_PROTOTYPE.md`](ACKERMANN_CONTROLLER_PROTOTYPE.md) for
equations, assumptions, full test metrics, and integration prerequisites.
