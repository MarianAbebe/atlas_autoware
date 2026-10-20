# Campus closed-loop simulation

Isolated Autoware 1.9.0 MPC lateral/PID longitudinal control on the real campus
map. Normal steering and acceleration come from the pinned controller. The
pinned command gate supplies emergency braking; the test watchdog publishes
only heartbeat, status, and a guarded rearm service.

The headless default uses the pinned `IDEAL_STEER_ACC_GEARED` model, DRIVE gear,
and command gate. The ungeared ideal model was freshly shown to retain creep
when the controller enters STOPPED and commands zero acceleration. No upstream
source, map, production Atlas configuration, or system package is changed.

```bash
cd ~/roam/atlas_autoware
# Reuses all cached dependencies and installs only the isolated demo changes:
bash src/atlas_closed_loop_demo/scripts/build_demo.sh
# Safe headless default; records a complete drive and ten-second terminal hold:
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --timeout 125
# Six independent interruption/recovery tests, sequentially in ROS domain 79:
python3 src/atlas_closed_loop_demo/scripts/run_safety_suite.py
# Explicit diagnostic baseline; expected to fail stopping checks:
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --ungated-baseline --vehicle-model IDEAL_STEER_ACC --timeout 115
# Optional graphical run, only in an accessible Ubuntu desktop session:
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --rviz --timeout 125
```

The actual two-process DDS probe determines readiness; a UDP socket diagnostic
alone does not decide whether DDS delivery works. The runner does not require
sudo, network reconfiguration, package installation, or unrestricted execution.
RViz availability is checked separately and is not required for headless tests.

Individual fault tests use `--fault drop|freeze --fault-input
odometry|steering|command`. Original ROS timestamps are preserved. Real simulator
state is recorded separately from interrupted controller inputs. Inputs recover
after eight seconds, followed by five seconds of observation without rearming.
The monitor requires continuously fresh inputs for one second before initial
arming and remains latched after any armed input exceeds 0.3 seconds of age.

For an interactive simulation, rearming requires valid inputs and three seconds
at standstill. Call `/atlas_closed_loop/feedback_watchdog/rearm` with
`std_srvs/srv/Trigger`, wait for fresh armed status, and separately call the pinned
`/atlas_closed_loop/gate/vehicle_cmd_gate/service/clear_external_emergency_stop`
Trigger service. Neither action is automatic. These are simulation-only topics
in domain 79; no physical vehicle interface is launched.

The initial reference is 70.279 m / 226 points and includes 0.5 m behind the route
start. Within 10 m of the goal, the demo increases the pinned planner's rearward
extent to 5 m to retain sufficient spline support. It continues using real live
odometry and the same forward route; no odometry or goal alignment is changed.

Provisional wheelbase is 1.20 m. The model is ideal and noise-free; these tests
do not validate physical Atlas dynamics or production safety. Recording closes
before cleanup disengagement/unloading, so cleanup cannot count as a stop.

See [VALIDATION.md](VALIDATION.md) for fresh numerical evidence, thresholds,
known shutdown warnings, visual status, and remaining limitations.
