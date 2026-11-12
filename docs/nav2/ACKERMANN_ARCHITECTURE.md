# Atlas Ackermann navigation fixture

The new graph is isolated from Autoware and physical actuators:

```text
Smac Hybrid-A* → Atlas AckermannController (Nav2 Humble plugin) → gated Twist
  → Ackermann adapter → Control(velocity, steering angle/rate, acceleration)
  → constrained bicycle simulator → odom/TF/encoder feedback
```

`ackermann_adapter` uses `steering = atan(wheelbase * yaw_rate / velocity)`
only above a configured low-speed threshold. A near-zero-speed angular request
is explicitly rejected and produces zero speed/steering; it is not represented
as a rotate-in-place command. The
simulator follows `yaw_rate = velocity * tan(steering) / wheelbase`; therefore
zero speed cannot change yaw.

All current values (`wheelbase=1.20 m`, steering limit `0.50 rad`, speed
`0.80 m/s`) are simulation placeholders, not Atlas measurements. Steering-rate,
acceleration/deceleration, speed, and command-timeout limits are enforced. A
timed-out command targets zero speed using bounded deceleration; this is not a
physical stopping-distance model.

The live custom-controller launch selects Smac Hybrid-A* with forward-only
Dubins search and a 2.25 m minimum turning radius. The simulated vehicle limit
is about 2.196 m; the planner margin is intentional. This launch does not plan
reverse cusps or rotate in place. Historical RPP/MPPI parameter files and the
skid-steer fixture remain available for comparison and are not part of this
Ackermann runtime result.

`synthetic_laser` ray-casts the checked-in scenario geometry and publishes
`/atlas_ackermann/scan` in `laser`; Nav2's local obstacle layer consumes that
scan. The planner does not receive simulator obstacle geometry directly. The
same geometry supplies an ideal contact check in the simulator, so an attempted
collision stops simulated motion rather than allowing an invalid pose.

The Humble stock behavior tree includes recovery Spin.  The Ackermann launch
uses `behavior_trees/ackermann_navigate_to_pose.xml` for `NavigateToPose`, so
that action can only compute a path and follow it; it cannot request recovery
spin or backup.  `behavior_server` remains only to let Humble initialise its
unused `NavigateThroughPoses` action.  An external request for a zero-speed
rotation is still converted to a safe zero command by the adapter.

The adapter publishes `/atlas_ackermann/command_status` with requested and
applied speed, steering, and curvature. A curvature beyond the steering limit
is rejected to zero instead of silently clipped. Rejection logs are throttled
to one per second and include requested curvature and steering; the per-command
status stream retains all values. This makes an infeasible controller request
visible without flooding the constrained VM's logs.

`synthetic_laser` stamps each scan with the timestamp of the odometry pose used
to ray-cast it. It does not stamp an older pose with the later timer callback
time. If odometry is delayed, the scan therefore remains honestly timestamped
and TF/message-filter handling can reject it as stale.

## Live Nav2 Ackermann controller

`src/atlas_nav2_demo/src/ackermann_controller.cpp` implements the Humble
`nav2_core::Controller` plugin `atlas_nav2_demo::AckermannController`. The
isolated launch is `ros2 launch atlas_nav2_demo nav2_ackermann_controller.launch.py
scenario:=<open|curved|avoidable|blocked>`. Parameters are in
`config/ackermann_controller_nav2_params.yaml`; the launch is separate from
the RPP and MPPI configurations.

For each candidate lookahead, the controller transforms the plan into the
costmap global frame, projects the vehicle onto the path, interpolates a
target, and calculates pure-pursuit curvature `kappa = 2*y_local /
(x_local^2+y_local^2)`. Steering is `delta = atan(wheelbase*kappa)` and Twist
angular velocity is `omega = v*tan(delta)/wheelbase`. It chooses the shortest
preview within the configured steering/curvature bound; if none is feasible,
it outputs zero rather than clipping steering. The current search spans
0.20–2.00 m in 0.05 m increments. Speed, acceleration, steering rate, and
steering magnitude use explicit simulation assumptions.

Before motion, it checks the full current costmap footprint plus a 0.05 m pad,
then rolls out command-reaction and braking distance using a 0.37 s reaction
assumption, 1.20 m/s² deceleration assumption, 0.10 s control period, and
0.05 m spatial sampling. It also checks the proposed next control interval.
Swept footprint samples account for translation and rotation, and motion
requires a current costmap. Invalid/stale pose, TF/costmap errors, or an
occupied swept footprint produce zero. The costmap uses a 0.32 m circular
footprint; this is not measured vehicle geometry. The simulator uses bounded
deceleration; these assumptions do not establish physical stopping distance.

The Humble controller-server `odom_topic` is explicitly
`/atlas_ackermann/odom`. Without it, OdomSmoother supplied zero speed while the
simulator was moving, distorting acceleration and braking predictions. The
safety gate remains downstream authority and the adapter still rejects
infeasible curvature. `map -> odom` is a simulation-only identity; the
simulator alone publishes `odom -> base_link` and odometry.

The Python controller prototype remains a reference implementation; its tests
are not live Nav2 evidence. Current runtime results and limitations are in
[`VALIDATION.md`](VALIDATION.md) and
[`NAV2_INTEGRATION_STATUS.md`](NAV2_INTEGRATION_STATUS.md).

The live plugin also checks LaserScan availability and freshness before
generating a nonzero command. Its configurable scan topic defaults to
`/atlas_ackermann/scan`; the source header stamp must be valid and within the
configured 0.30 s timeout, and local receipt age is measured monotonically.
Missing/stale scan data produces zero command while preserving the costmap
collision check. This is a fail-stop check, not a replacement for Nav2's
obstacle layer.

The simulator publishes `/atlas_ackermann/simulator_dynamics` once per model
step. Pose, speed transition, acceleration, steering transition, steering
rate, and `integration_dt_sec` share its simulation timebase. Odometry headers
and callback receipt times remain separate; they are not used to claim exact
integrator acceleration. `SetBool` test services can pause odometry/TF or
synthetic LaserScan output for bounded fault injection. These are test controls
in the isolated fixture, not deployment interfaces.

The safety gate, adapter, simulator, and Nav2 controller server run as separate
ROS processes. The Python gate and simulator use monotonic clocks for command
age, and both fail closed after they resume from a missed timer interval. They
still share the host OS and VM scheduler. A retained curved-navigation trace
recorded 731 ms between gate ticks and 758 ms between simulator ticks; on
resumption the gate rejected stale inputs and published zero, and Nav2 aborted
safely. This does not establish behavior during the pause on physical actuators
and is not an independent hardware emergency-stop path. Timing evidence and the
current open acceptance status are in [`NAV2_INTEGRATION_STATUS.md`](NAV2_INTEGRATION_STATUS.md).

The default RPP minimum lookahead of 0.35 m leads the adapter to reject
infeasible steering requests while stationary. Additional fixed 1.2 m and
1.8 m probes still requested steering above the 0.50 rad limit. A 2.0 m fixed
lookahead produced feasible commands and moved the vehicle, but the controller
predicted a collision close to an obstacle corner after the actual vehicle had
diverged from the global path. The recorded projected clearance was about
0.035 m after the 0.32 m simulator footprint, so the collision stop is
plausible and remains enabled.

Nav2 Humble MPPI has an Ackermann motion model, but isolated probes showed
published commands beyond the configured curvature limit; disabling reverse
sampling did not eliminate the violation. The vehicle adapter rejected those
commands and the run aborted on progress. Neither RPP nor MPPI has passed the
avoidable-obstacle acceptance test in the current configuration. Do not
promote either as validated Atlas navigation or count any safe abort as a
success. Detailed synchronized traces, probe configuration, timing, and next
engineering action are in `VALIDATION.md` and
`validation_evidence/ackermann/controller_eval_20261109/`.

The runtime recorder is `scripts/record_ackermann_navigation.py`; it captures
the path, controller/gated commands, adapter and gate status, odometry, scan,
costmaps, TF, action state, and controller warnings with ROS and monotonic
receipt times. Use it for future trials rather than inferring tracked motion
from a planned path alone.

The isolated Python tracker remains as a reference implementation for
deterministic parity tests. It is not connected to the live graph. The C++
plugin and live validation status are described above and in
[`NAV2_INTEGRATION_STATUS.md`](NAV2_INTEGRATION_STATUS.md).
