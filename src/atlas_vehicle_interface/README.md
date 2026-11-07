# Atlas vehicle-interface boundary

This package remains intentionally free of a physical motor-driver protocol.
The reusable simulation references live in `atlas_nav2_demo`.  Its four-wheel
skid-steer mixer is retained as a historical isolated fixture, not an Atlas
drivetrain model.  Atlas's intended path is the Ackermann adapter, bicycle
simulator, ideal rear-axle encoder feedback, and safety gate; neither fixture
is a physical vehicle driver.

## Future Jetson-to-vehicle contract

The intended Jetson-side adapter must accept a safety-gated command containing
longitudinal velocity, steering angle, optional acceleration, and optional
steering rate (currently `autoware_control_msgs/Control`). It must emit
separate traction and steering actuator targets. Wheelbase, steering-rack
ratio, steering limits/rate, wheel radius, target timeout, encoder resolution,
and any slip model remain configuration inputs until measured.

The old skid-steer fixture accepted a gated body command `(v, omega)` and
emitted four wheel angular-velocity targets:

```text
left  = (v - omega * track_width / 2) / wheel_radius
right = (v + omega * track_width / 2) / wheel_radius
front_left = rear_left = left
front_right = rear_right = right
```

That relationship is explicitly not the Atlas physical-interface contract.

Required feedback is left/right wheel position and velocity, with acquisition
timestamps and explicit validity/fault status. The future odometry estimator
must reject invalid or stale feedback rather than substitute simulator truth.
The actuator transport must zero targets on command timeout, rejected input,
shutdown, or an asserted safety stop. It must never automatically clear a
latched safety fault.

## Current evidence

`atlas_nav2_demo/test/test_ackermann_kinematics_unittest.py` covers straight,
reverse, curved, bounded-radius, saturation, and no-in-place-rotation bicycle
motion. The safety-contract tests cover stale command, stale localization,
disabled gate, and emergency-stop zero output. Encoder dropout and real
motor-controller communication remain hardware-interface work.
