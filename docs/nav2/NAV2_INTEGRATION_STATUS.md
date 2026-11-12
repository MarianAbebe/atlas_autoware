# Ackermann Nav2 integration status

Updated 2026-11-11 after the final timing investigation. This is an isolated
simulation; the geometry, dynamics, and timings below are not measured Atlas
hardware properties.

## Completion decision

**Navigation validated; timing acceptance remains open.** Straight navigation
passed 5/5, avoidable-obstacle navigation passed 5/5, and a fresh consecutive
curved campaign passed 5/5. A separate five-run curved campaign had one safe
abort after a source-timestamped multi-stream execution interruption; that
failure is retained. The blocked route aborted safely. The six full-graph
fault cases and 70 package tests passed after adding portable overlay checks. A
20-trial cancellation campaign measured gate-generated safe zero within
69.1–106.5 ms (median 79.4 ms), and
simulator application within 76.0–110.6 ms (median 90.3 ms), all below the
250 ms freshness limit. However, the earlier 363.8 ms observer-receipt outlier
cannot be reconstructed stage-by-stage because that capture lacked
publisher-side monotonic timestamps. More recent navigation evidence confirms
that the gate and simulator timers can themselves experience a shared
wall-clock scheduling interruption. Therefore these 20 cancellation trials do
not establish response timing during such an interruption; the safety timing
criterion remains open. No threshold was changed.

No hardware validation has been performed. Finite tests do not establish a
hard real-time guarantee.

## Implementation and command path

The active development launch is:

```bash
source /opt/ros/humble/setup.bash
source "$ATLAS_NAV2_OVERLAY/setup.bash"
export ROS_DOMAIN_ID=211 ROS_LOCALHOST_ONLY=1
ros2 launch atlas_nav2_demo nav2_ackermann_controller.launch.py scenario:=open
```

The graph is Smac Hybrid-A* (forward-only Dubins, 2.25 m minimum radius) →
Humble `nav2_core::Controller` plugin → gated Twist → Ackermann adapter →
`autoware_control_msgs/Control` → bounded bicycle simulator → odometry/TF.
RPP and MPPI configurations remain separate and unchanged. The safety gate
and adapter remain authoritative. The plugin now also stops when its configured
LaserScan is absent, has an invalid/old source stamp, or has exceeded its
monotonic receipt timeout (0.30 s); it still requires a current local costmap.

Simulation assumptions: wheelbase 1.20 m, steering limit 0.50 rad (maximum
curvature about 0.4553 1/m), steering rate 0.60 rad/s, speed limit 0.80 m/s,
acceleration 0.80 m/s², deceleration 1.20 m/s², command timeout 0.25 s, and
robot footprint radius 0.32 m. Navigation controller speed is 0.20 m/s. Its
collision rollout assumes 0.37 s reaction time, 0.10 s control period, and
0.05 m additional margin. These are fixture settings, not measured hardware
limits. Braking is a bounded simulated deceleration, not instantaneous stop or
physical stopping-distance validation.

## Final-code navigation regression

The acceptance runner launched a clean Nav2 graph for each trial on sequential,
isolated DDS domains; it required active lifecycle nodes, a successful action,
stopped terminal pose within 0.20 m / 0.25 rad, feasible adapter samples, and
no E-stop. All three successful scenarios passed five consecutive runs.

| Scenario | Attempts | Pass | Results |
|---|---:|---:|---|
| Straight | 5 | 5 | XY error 0.1683–0.1712 m; yaw error <0.001 rad; max steering 0.000003 rad; no infeasible adapter command. |
| Curved | 5 consecutive follow-up passes; prior set 4/5 | 5 in follow-up set | XY error 0.1650–0.1704 m; yaw error 0.0721–0.0749 rad; cross-track 0.0103–0.0114 m; max steering 0.49985–0.49996 rad; steering rate ≤0.60000003 rad/s. The preceding failed run safely aborted during the documented timer interruption. |
| Avoidable obstacle | 5 | 5 | XY error 0.1601–0.1792 m; yaw error 0.0214–0.0509 rad; cross-track 0.0338–0.0471 m; obstacle clearance 0.6379–0.6476 m; max steering 0.49968–0.49999 rad; no simulator collision clamps or adapter rejections. |
| Blocked route | 1 | 1 safe abort | Action `ABORTED`; 60 odometry samples; displacement ≤0.02 m; 65 safe-command samples all zero; no E-stop. This is an expected safe abort, not a navigation success. |

Per-run synchronized simulator diagnostics show maximum integration-step
acceleration +0.8000000000000006 m/s² and minimum -1.2000000000000006 m/s².
The simulator dynamics channel uses the same integration step as its state
update; this is authoritative for model compliance. ROS odometry header and
receipt times are separately recorded and must not be used as if they were
aligned integrator timestamps. Derived values are in
`~/roam/validation_evidence/ackermann/final_acceptance_20261111/nav_regression/derived_dynamics_metrics.json`. The timing-investigation regression
captures and summaries are in
`~/roam/validation_evidence/ackermann/final_acceptance_20261111/timing_investigation/nav_retest/`.

## Full-graph safety fault injection

`validate_ackermann_faults.py` ran the actual planner, controller server,
adapter, gate, and simulator. The six-case summary is
`~/roam/validation_evidence/ackermann/final_acceptance_20261111/full_graph_faults_final/fault_summary.json`.

| Fault | Result | Detection / first safe zero | Simulated travel to standstill / envelope |
|---|---|---:|---:|
| Odometry/localization loss | Pass; stale detected, stop, sensor restored, motion recovered, then goal canceled | 243 / 243 ms | 0.0532 / 0.1056 m |
| LaserScan loss | Pass; controller stale-scan stop, scan restored, navigation resumed, then canceled | 266 / 271 ms | 0.0636 / 0.1208 m |
| Controller command loss | Pass; gate timed out and action aborted | 201 / 201 ms | 0.0380 / 0.0875 m |
| Emergency stop | Pass; latched; clearing alone stayed disabled; explicit rearm required; old goal did not resume | 8 / 4 ms | 0.0029 / 0.0542 m |
| Navigation cancellation | 20/20 bounded trials met 250 ms gate safe-zero requirement; separate historical outlier retained | action result median 24.8 ms; gate zero median 79.4 ms / max 106.4 ms; simulator target median 90.3 ms / max 110.6 ms | standstill median 193.9 ms / max 230.5 ms; travel max 0.0232 m, inside all modeled envelopes |
| Non-finite command | Pass; latched E-stop, zero output, clear did not re-enable, explicit rearm did not resume old goal | 13 / 1 ms | 0.0029 / 0.0539 m |

For the historical 363.8 ms cancellation outlier, action cancellation was
prompt, but safe-zero receipt and all major graph streams paused together for
about 0.33 s. That capture lacked publisher-side monotonic timestamps, so the
stage where the delay occurred cannot be established. The new 20-run capture
records internal and external monotonic times: gate publication was 69.1–106.4
ms (median 79.4 ms), safe-zero observation 69.7–106.8 ms (median 79.7 ms),
adapter publication 70.2–107.2 ms (median 80.2 ms), and simulator target
application 76.0–110.6 ms (median 90.3 ms). Simulated standstill occurred in
176.0–230.5 ms (median 193.9 ms), with 0.0148–0.0232 m travel against a
0.0688–0.0778 m modeled envelope. All 20 cancellations met the 250 ms
safe-command limit.

This does not resolve the separate scheduling interruption. In the retained
curved-navigation failure, embedded gate timer timestamps had a 731 ms gap and
embedded simulator tick timestamps had a 758 ms gap. On resumption, the gate
reported stale command and localization and emitted zero; the simulator saw a
command age over 0.8 s and integrated only its capped 50 ms step. The vehicle
was already stopped at that point. The Nav2 action then safely aborted after
repeated stale-pose/scan and infeasible-preview warnings. Because there were no
per-process resource samples on that run, the cause cannot be attributed to
CPU, memory, executor contention, DDS, or QEMU scheduling. The independent gate
and simulator are separate processes and use monotonic timeout clocks, but they
share the same OS/VM and are not independent real-time safety hardware.

The 20 cancellation trials had no threshold exceedance. Internal callback
interarrival gaps stayed below 23 ms in this campaign, although diagnostic
recorder delivery lag reached 392 ms. Resource sampling found 19.4% median and
78.1% maximum aggregate CPU utilization across six vCPUs, at least 1.35 GiB
available memory, and no swap-out increase. The planner briefly reached 99.3%
of one core; the controller peaked at 31.2% of one core. These samples were
from cancellation runs, not the curved interruption, and do not identify its
cause. The 250 ms freshness threshold was not changed;
timing acceptance remains open pending a targeted source-timestamped capture
with per-process scheduling/resource evidence under native Ubuntu or equivalent
isolated scheduling conditions.

The measured envelope uses speed before safe zero × observed monotonic response
latency + `v²/(2 × 1.20 m/s²)` + 0.05 m. It is a simulator assumption and
does not validate physical braking. Localization/scan recovery is automatic
only after fresh healthy data; E-stop and invalid-command latches require
explicit clear and enable; the previous goal remains canceled.

## Build and regression

```bash
source /opt/ros/humble/setup.bash
export ATLAS_NAV2_OVERLAY="${ATLAS_NAV2_OVERLAY:-$PWD/.nav2_cache/ackermann/install}"
build_base="$(dirname "$ATLAS_NAV2_OVERLAY")/build"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --base-paths src/atlas_nav2_demo --packages-select atlas_nav2_demo \
  --build-base "$build_base" --install-base "$ATLAS_NAV2_OVERLAY" \
  --executor sequential --cmake-args -DBUILD_TESTING=ON
source "$ATLAS_NAV2_OVERLAY/setup.bash"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon test \
  --base-paths src/atlas_nav2_demo --packages-select atlas_nav2_demo \
  --build-base "$build_base" --install-base "$ATLAS_NAV2_OVERLAY" \
  --executor sequential
colcon test-result --test-result-base "$build_base/atlas_nav2_demo" --verbose
```

Result: build succeeded; 70 tests, 0 errors, 0 failures, 0 skipped. Final-code
navigation commands (headless) were:

```bash
ROS_LOCALHOST_ONLY=1 python3 src/atlas_nav2_demo/scripts/run_ackermann_acceptance.py \
  --scenario straight --repetitions 5 --domain-start 150 \
  --evidence-root ~/roam/validation_evidence/ackermann/final_acceptance_20261111/timing_investigation/nav_retest/straight
ROS_LOCALHOST_ONLY=1 python3 src/atlas_nav2_demo/scripts/run_ackermann_acceptance.py \
  --scenario curved --repetitions 5 --domain-start 155 \
  --evidence-root ~/roam/validation_evidence/ackermann/final_acceptance_20261111/timing_investigation/nav_retest/curved
ROS_LOCALHOST_ONLY=1 python3 src/atlas_nav2_demo/scripts/run_ackermann_acceptance.py \
  --scenario curved --repetitions 5 --domain-start 196 \
  --evidence-root ~/roam/validation_evidence/ackermann/final_acceptance_20261111/timing_investigation/nav_retest/curved_followup
ROS_LOCALHOST_ONLY=1 python3 src/atlas_nav2_demo/scripts/run_ackermann_acceptance.py \
  --scenario avoidable --repetitions 5 --domain-start 190 \
  --evidence-root ~/roam/validation_evidence/ackermann/final_acceptance_20261111/timing_investigation/nav_retest/avoidable
ROS_LOCALHOST_ONLY=1 python3 src/atlas_nav2_demo/scripts/run_ackermann_acceptance.py \
  --scenario blocked --repetitions 1 --domain-start 195 \
  --evidence-root ~/roam/validation_evidence/ackermann/final_acceptance_20261111/timing_investigation/nav_retest/blocked
```

Use a clean graph between trials and avoid RViz. The runner checks that the
development overlay resolves. Full fault reproduction:

```bash
ros2 run atlas_nav2_demo validate_ackermann_faults.py --fault all \
  --domain-start 220 \
  --evidence-root ~/roam/validation_evidence/ackermann/final_acceptance_20261111/timing_investigation/final_fault_regression
```

## Native Ubuntu reproduction

A native Ubuntu run has not been performed. The complete Jammy/Humble setup,
branch checkout, dependency installation, portable overlay build, headless
navigation and cancellation campaigns, process resource sampling, and
acceptance criteria are maintained in
[`NATIVE_UBUNTU_VALIDATION.md`](NATIVE_UBUNTU_VALIDATION.md). The entry point is
`tools/nav2/run_ackermann_native_validation.sh`. Native Ubuntu is a test
platform, not an assumed fix for the scheduling interruption.

## Acceptance decision and next work

Navigation behavior, obstacle avoidance, sensor-loss stops, command timeout,
E-stop latch/rearm, malformed command rejection, and exact simulator dynamics
are implemented and demonstrated. The current milestone outcome is **B:
navigation validated; timing acceptance open**. The repeated cancellation
check passed 20/20 under the 250 ms limit, but the observed multi-process
timer interruption means timing under a shared scheduling stall is not
established. Do not increase timeouts or weaken safety checks. Physical wheel
geometry, braking, steering dynamics, sensors, localization, and hardware
command interfaces remain unvalidated. Physical deployment will require an
actuator-side command timeout and an appropriate hardware E-stop independent of
this ROS process graph. The next action is a native-Ubuntu
reproduction with source-monotonic callbacks and per-process scheduler/resource
traces, followed by the same cancellation and fault checks; native Ubuntu is a
test environment, not an assumed fix.
