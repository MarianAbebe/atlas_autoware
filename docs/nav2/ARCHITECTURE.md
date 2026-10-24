# Nav2 milestone architecture

```text
synthetic_map.pgm ──> Nav2 map server ──> global costmap ──> NavFn planner
                                                             │
ideal odom + TF ──────────────────────────────────────────────┤
synthetic LaserScan ──> local obstacle costmap ──> DWB controller
                                                             │ Twist
                                                             v
                                                  simulation safety gate
                                                             │ gated Twist
                                                             v
                                                   four-wheel mixer
                                                             │ wheel rad/s
                                                             v
                                              ideal skid-steer simulator
                                                   │ odom, TF, encoders
                                                   └──────────────> Nav2
```

## Ownership and frames

The simulator is the only publisher of `odom -> base_link`. It also publishes
the static identity `map -> odom` transform and static `base_link -> laser`
transform. Identity localization is an explicit simulation assumption for this
milestone; AMCL, GNSS, LIO, VIO, and factor-graph fusion are absent.

The laser node ray-casts the same boundary and central-obstacle geometry encoded
in the checked-in occupancy map. Messages use current ROS timestamps and the
`laser` frame. Nav2's global costmap consumes the static occupancy map; its local
costmap consumes the laser scan through an obstacle layer.

## Command safety boundary

Nav2 publishes only to `/atlas_nav2/cmd_vel_nav`. The safety gate is the sole
publisher of `/atlas_nav2/cmd_vel_safe`; it starts disabled, rejects non-finite
commands by entering emergency stop, clamps velocity, and outputs zero for stale
commands, disabled state, emergency stop, and shutdown. Clearing emergency stop
does not enable the gate.

The wheel mixer implements:

```text
left  = (v - omega * track_width / 2) / wheel_radius
right = (v + omega * track_width / 2) / wheel_radius
```

Front/rear wheels on each side receive equal ideal targets. The simulator
inverts these equations, integrates a planar no-slip pose, publishes wheel
position/velocity as ideal encoder feedback, and enforces its own target timeout.

## Scope boundaries

The model has no wheel slip, soil interaction, motor torque/current, suspension,
latency model, actuator dynamics, or outdoor-traction fidelity. It is a software
integration fixture and cannot command physical hardware. Existing Autoware
watchdogs, emergency behavior, packages, and stationary validation are unchanged.
