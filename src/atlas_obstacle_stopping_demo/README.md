# Campus obstacle-stop simulation

This isolated package connects the real-campus loop to pinned Core 1.9.0
`autoware_motion_velocity_planner` with
`autoware::motion_velocity_planner::ObstacleStopModule`. Fresh measured results
and limitations are recorded in [VALIDATION.md](VALIDATION.md).

The existing closed-loop package is reused without changes. The launch is an
explicit variant of its launch; all controls still originate in pinned MPC/PID
and the pinned command gate. This package supplies synthetic perception inputs
and an authorization monitor, never a steering, acceleration or stop trajectory.
There is no physical vehicle interface. Use ROS domain **80**, separately from
the previous domain-79 experiments. Topic namespaces remain `/atlas_closed_loop`
and `/atlas_map_demo`, which must not be bridged to production.

## Dependency and build commands

From `~/roam/atlas_autoware`, with the existing durable caches present:

```bash
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
source .closed_loop_cache/demo_install/setup.bash
python3 src/atlas_obstacle_stopping_demo/scripts/audit_dependencies.py
```

The audit checks exact pinned revisions, clean upstream working trees, all
non-test manifest edges and system dependencies. It writes ignored evidence
under `results/`. The original missing system set contained only `ros-humble-grid-map-core`;
the user approved and installed it before the successful build. No script
installs packages. On a new host, obtain separate approval before this command:

```bash
# Only after approval; never run ROS nodes with sudo:
sudo apt-get --no-install-recommends --no-upgrade --no-remove install ros-humble-grid-map-core
```

After dependency resolution:

```bash
bash src/atlas_obstacle_stopping_demo/scripts/build_demo.sh
```

The build reuses `.closed_loop_cache`, builds only missing source packages in
ignored `.obstacle_cache`, then installs this launch/scripts package separately.
Nothing is installed into the OS or upstream checkouts by this build script.
The original map locations and provisional vehicle geometry are inherited from
the closed-loop demo; these remain workstation-specific experiments.

## Bounded numerical tests

```bash
# Fresh complete suite: three route runs, two perception faults, six feedback faults.
python3 src/atlas_obstacle_stopping_demo/scripts/run_suite.py
# Individual cases:
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario none
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario ahead
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario off_route
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario stale_drop
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario stale_freeze
bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh --scenario feedback_freeze --feedback-input steering
```

Each run copies the predeclared thresholds and source hashes, launches actual
pinned components, records true vehicle state, then evaluates numerical checks.
Normal and off-route cases have a 125 s bound; ahead/clearance has 160 s; each
fault case has 45 s. Cleanup is measured separately and cannot count as stopping.
The suite has a 210 s outer bound per child. Logs and raw recordings are local,
ignored under `results/`; JSON reports contain explicit failed checks. New suites
use fresh timestamped `results/suite_*` folders and automatically run the offline
evidence review; use `--results-root` to select another empty folder.
The `ahead` case includes both a five-second obstacle hold and normal clearance/
resumption, then finishes the campus route with a ten-second terminal hold.
Stale cases restore fresh data but deliberately withhold explicit rearm.

## Interactive development launch

```bash
bash src/atlas_obstacle_stopping_demo/scripts/run_demo.sh \
  results_dir:=/tmp/atlas_obstacle_development launch_rviz:=false
```

This launch runs until Ctrl-C and is **not** the bounded numerical test suite.
Do not use it as evidence of passed stopping/clearance tests. Use the bounded scripts above for numerical results. Predeclared limits are
in `config/thresholds.json`; the validation report identifies the evaluated runs.

The fixture starts with fresh empty clouds and an explicitly empty required
PredictedObjects stream. Its injection interface is
`/atlas_closed_loop/obstacles/inject` (`std_msgs/msg/String`), JSON:

```json
{"mode":"obstacle", "x":0.0, "y":0.0, "z":0.0, "yaw":0.0}
```

Coordinates above are schema examples, **not** the campus obstacle location.
Choose the actual anchor from the measured route in `map`; the block spans
0.8 m longitudinally and 0.6 m laterally with points 0.1–0.4 m above anchor z.
It publishes 252 XYZ points at 20 Hz. `{"mode":"clear"}` publishes fresh empty
clouds; `freeze` repeats the last original cloud including its timestamp;
`drop` stops cloud delivery. PredictedObjects remains explicitly empty and fresh.
These are perception fixtures, not commands to the vehicle.

## Safety recovery

Normal clearance keeps input timestamps advancing and does not require a safety
reset. A required-input freshness fault must latch authorization off. The
original feedback watchdog remains in the loop; the new monitor adds perception
freshness before forwarding authorization to the pinned gate. Recovery alone
does not rearm. While stale, rearm must be refused. Explicit recovery requires
fresh inputs and at least three seconds of live standstill:

1. If the original feedback watchdog latched, explicitly rearm it first through
   `/atlas_closed_loop/feedback_watchdog/rearm` (`std_srvs/srv/Trigger`).
2. Explicitly rearm `/atlas_closed_loop/obstacle_guard/rearm` with that type.
3. After fresh armed status, separately clear the pinned gate through
   `/atlas_closed_loop/gate/vehicle_cmd_gate/service/clear_external_emergency_stop`.

Measured gate braking, hold, invalid-rearm refusal and recovery without automatic
restart are reported separately from normal obstacle clearance. Positive driving
after a valid explicit fault reset is not part of the current test suite. These
provisional ideal-model tests do not establish physical vehicle safety.

## Watchdog diagnosis

The 2026-10-22 investigation retained first-latch age snapshots in both safety
status messages and added a passive runtime tracer plus pure decision tests.
Thresholds and latch/rearm behavior are unchanged. The latest complete diagnostic
suite is **9/11**, so the no-regression criterion is unresolved; the older passing
results above remain historical evidence. See
[the watchdog investigation](../../docs/regressions/obstacle_watchdog/REPORT.md)
for exact timestamps, failed checks, experiments, reproducible commands and the
remaining delayed-hop question. No affinity/thread-limit workaround was retained.
