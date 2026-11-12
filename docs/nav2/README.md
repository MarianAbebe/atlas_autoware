# Atlas Nav2 simulation

Atlas uses Ackermann steering. The current isolated simulation uses the custom
ROS 2 Humble `nav2_core::Controller` plugin with Smac Hybrid-A*, a bicycle-model
simulator, synthetic LaserScan input, live costmap checks, adapter, and
default-disabled safety gate. It does not launch Autoware and has no physical
actuator connection. `nav2_skid_steer.launch.py` is retained only as a
historical regression fixture.

## Current status

Straight, curved, and avoidable-obstacle navigation each have a five-run
passing campaign; the blocked route aborts safely. Full-graph fault injection
passed 6/6, package regression passed 70 tests, and 20 cancellation trials
produced safe zero within the configured 250 ms window. One other curved run
recorded 731 ms gate and 758 ms simulator timer gaps and safely aborted. The
cause remains unresolved. Status:
**navigation validated; timing acceptance open**. This is not hardware
readiness or a real-time safety guarantee.

See [Ackermann architecture](ACKERMANN_ARCHITECTURE.md),
[validation results](VALIDATION.md), and
[integration status](NAV2_INTEGRATION_STATUS.md).

## Native Ubuntu validation

For Ubuntu 22.04 and ROS 2 Humble dependencies, a clean development-branch
checkout, isolated overlay build, headless acceptance scenarios, cancellation
timing campaign, resource capture, and pass/fail criteria, follow
[NATIVE_UBUNTU_VALIDATION.md](NATIVE_UBUNTU_VALIDATION.md). The single campaign
entry point is:

```bash
source /opt/ros/humble/setup.bash
tools/nav2/run_ackermann_native_validation.sh
```

It builds only `atlas_nav2_demo`, runs its tests, then runs straight (5), curved
(5), avoidable obstacle (5), blocked route (1), all six full-graph fault cases,
and 20 sequential cancellation trials. It does not launch RViz. Evidence is
written outside source directories by default.

The active launch can also be started directly after building and sourcing the
development overlay:

```bash
source /opt/ros/humble/setup.bash
source "$ATLAS_NAV2_OVERLAY/setup.bash"
export ROS_LOCALHOST_ONLY=1
ros2 launch atlas_nav2_demo nav2_ackermann_controller.launch.py scenario:=open
```

The gate starts disabled. Prefer the automated test runner, which manages gate
enable/disable, goals, evidence, isolated domains, and graph cleanup.

## Preserved skid-steer fixture

The older Nav2 demo includes a skid-steer launch, wheel mixer, and differential
model. It is not representative of Atlas's confirmed drivetrain and its
validation must not be cited as Atlas Ackermann acceptance.
