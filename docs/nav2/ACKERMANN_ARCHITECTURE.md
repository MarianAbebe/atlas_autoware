# Atlas Ackermann navigation fixture

The new graph is isolated from Autoware and physical actuators:

```text
Smac Hybrid-A* → Regulated Pure Pursuit → gated Twist
  → Ackermann adapter → Control(velocity, steering angle/rate, acceleration)
  → constrained bicycle simulator → odom/TF/encoder feedback
```

`ackermann_adapter` uses `steering = atan(wheelbase * yaw_rate / velocity)`
only above a configured low-speed threshold. Near zero speed it emits zero
steering demand rather than representing a rotate-in-place command. The
simulator follows `yaw_rate = velocity * tan(steering) / wheelbase`; therefore
zero speed cannot change yaw.

All current values (`wheelbase=1.20 m`, steering limit `0.50 rad`, speed
`0.80 m/s`) are simulation placeholders, not Atlas measurements. Steering-rate,
acceleration/deceleration, speed, and command-timeout limits are enforced. A
timed-out command targets zero speed using bounded deceleration; this is not a
physical stopping-distance model.

Smac Hybrid-A* is selected with Reeds-Shepp nonholonomic search and a 2.20 m
minimum turning radius; reverse is therefore planned only as a bounded vehicle
maneuver, never as a turn-in-place substitute. Regulated Pure Pursuit is
configured without rotate-to-heading, with reversing allowed and curvature
regulation. The historical DWB/skid-steer
fixture remains intact but must not be cited for Ackermann acceptance.

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
applied steering/curvature. A curvature beyond the steering limit is rejected
to zero instead of silently clipped. This makes an infeasible controller path
visible as a navigation failure rather than a false successful trajectory.
