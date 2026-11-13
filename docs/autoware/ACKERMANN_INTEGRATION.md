# Autoware Ackermann Integration

## Status

This is a development simulation integration, not a physical vehicle interface.
The lanelet route, synthetic sensor scene, NDT/EKF localization, Autoware
velocity smoother, MPC/PID controller, command gate, and rate-limited Ackermann
simulator are connected in a closed loop. The 2026-11-12 repeatability campaign
completed three of six one-lap attempts; three runs safely latched freshness
watchdogs. Five consecutive route successes and five-lap operation have not
been demonstrated. A full-graph E-stop/clear/no-auto-resume check passed after
adding a disarm-on-E-stop safeguard. Other requested full-graph fault cases
remain open.
The simulator is the vehicle endpoint; no motor or steering hardware protocol
is implemented. No hardware validation has occurred.

## Timing acceptance follow-up — 2026-11-12

**Outcome: navigation validated; timing acceptance open.** The corrected
five-lap diagnostic run in
`/home/marian/roam/validation_evidence/autoware_ackermann/timing_acceptance_20261112T1705Z/full_5lap_attempt1`
latched safely after 2.859 laps (197.19 m of simulator-truth travel). At the
watchdog evaluation, trajectory and NDT source ages were 306.9 ms and 324.9 ms
against the unchanged 300 ms threshold. Their receipt ages were 129.6 ms and
129.2 ms, respectively. Raw-cloud source/receipt ages were 143.5/99.5 ms.
Thus the fault was source timestamp freshness for the NDT/trajectory chain,
not stale raw-cloud receipt at that instant. The corresponding source stamps
were about 18 ms apart, indicating they shared an older localization/planning
time window; it does not identify which producer first delayed the update.
The watchdog itself reported an NDT receipt gap peak of 816.8 ms over the run.
Its last NDT execution was 12.49 ms, and the maximum callback duration was
45.96 ms; a long NDT calculation is not established as the cause. Stale NDT
and trajectory remain separate monitored inputs, though they were stale
together in this event.

The historical ROS-time discontinuity (about 1172.9 seconds of ROS time in
21.8 ms monotonic time) was not reproduced in the corrected run. Across 960
clock samples in this run, maximum ROS/monotonic delta discrepancy was
16.75 microseconds and maximum wall/monotonic discrepancy was 262.5
microseconds. NTP was active in the historical guest log, but whether NTP,
QEMU/guest synchronization, or another event caused that earlier jump remains
unknown. A deterministic injected-clock unit suite covers forward/backward
jumps, ordinary scheduler delay, sticky latching, and explicit standstill
rearm; it does not change the host clock. The watchdog timer now uses a steady
clock while message source-age checks remain in the ROS clock domain. No
freshness thresholds were changed.

### Timing instrumentation and corrective changes

The initial 17:00Z smoke capture is retained but excluded from navigation
acceptance: a recorder indentation defect created duplicate subscriptions on
each 250 ms sample. The corrected five-lap capture had no such defect. A
separate timing review found the validator repeatedly reread the entire growing
launch log and rebuilt large route/planner path summaries at message rate.
The recorder now reads only appended log bytes and samples each path stage at
most once per second. This reduces measurement overhead; the change has not yet
been exercised in another full route campaign.

The corrected watchdog snapshot recorded a 688.99 ms maximum input callback
duration and 687.07 ms prior tick duration on its 50 Hz safety process. The
latched decision had already been made and heartbeat withheld before the
detailed warning was emitted. Inline formatting/logging occurred on that
critical callback, making it an avoidable source of callback delay. Detailed
fault logging is now deferred to the separate 10 Hz diagnostics callback; the
structured status still carries the fault details. This is a concrete software
improvement, but its effect on full-graph timing has not yet been runtime
validated. It cannot guarantee execution during an OS scheduling stall.

The watchdog status message itself was observed by the Python collector about
2.43 seconds after its embedded fault stamp. Component-side monotonic timing
maps the fault decision to approximately 77570.446 s and the scene callback at
that time to approximately 77570.448 s; the collector receipt field is later
because the validator's ROS callbacks were backlogged. Therefore validator
receipt order is not used as event order. The gate snapshot shows its own
watchdog tick interval at approximately 100 ms at fault evaluation, but a
separate maximum tick gap of 311.75 ms and a 687 ms previous watchdog tick
duration occurred elsewhere in the run. System-wide scheduling/processing
delay remains a plausible contributor, not a proven single cause of the
source-age lapse.

### Steering and acceptance status

The instrumented simulator emitted 2,304 internal steering integration steps;
2,303 had valid timesteps and the telemetry covered 100% of odometry samples.
The maximum internal rate was 0.5900000000000011 rad/s against the configured
0.59 rad/s cap (zero violations at a 1e-6 rad/s numeric tolerance). The largest
integration interval was 288.684 ms, so this verifies the model's rate limiter
at executed integration steps but does not establish real-time update cadence.
The downstream steering-report estimator had under 99% coverage and remains
inconclusive. This run did not complete five laps.

| Check | Evidence | Result |
|---|---|---|
| Clock-jump detection | Deterministic unit tests; 960 samples in corrected run | Unit behavior passed; no jump in this run; historical trigger unresolved |
| NDT/trajectory freshness | Fault snapshot and watchdog component timings | Safe latch; route interrupted at 2.859 laps |
| Internal steering rate | 2,303 valid steps in five-lap attempt and 3,682 in follow-up smoke | Both runs passed at 0.59 rad/s within 1e-6 tolerance |
| Continuous five-lap route | `full_5lap_attempt1` | Failed safely at 2.859 laps; no completion |
| Full-graph E-stop/clear/rearm | `acceptance_20261112T0310/estop_clear_retest` | Passed for tested path; no old-route resumption |
| Other full-graph fault types | No complete campaign | Open; isolated harness results are not full-graph evidence |
| Hardware timeout/E-stop | No physical hardware test | Not tested |

The watchdog and simulator execute as software processes subject to host/guest
scheduling. A timer-based software watchdog cannot promise an output while its
process is not scheduled. Physical deployment requires an independently
enforced actuator command timeout and a suitable hardware E-stop architecture;
neither is implemented or proven by a separate ROS process. The milestone
remains **navigation validated; timing acceptance open**, not hardware ready.

### Follow-up smoke after instrumentation changes

Evidence is retained at
`/home/marian/roam/validation_evidence/autoware_ackermann/timing_fix_20261112T/`.
The first one-lap smoke completed and held the goal, but is not an acceptance
run because the simulator diagnostic overlay was not in the sourced prefix;
internal steering-step samples were absent. The next run loaded both the
instrumented simulator and timing-fix overlays (prefixes were checked before
launch) and safely latched at 0.772 lap on `predicted_objects` freshness.
At the fault (ROS stamp 1794504292.0348766), predicted-object source age was
323.6 ms while its receipt age was 139.0 ms. Raw point-cloud source age was
122.9 ms and receipt age 83.3 ms. The last predicted-object sample carried
stamp 1794504291.711262; its adapter record measured 165.6 ms source age at
detection receipt, 232.9 ms between detection receipts, and 5.1 ms from
detection receipt to publication. The next sample with stamp 1794504291.911920
was observed after the fault. This supports an intermittent gap/delay upstream
of the adapter output; the adapter's measured processing time was small. It
does not yet isolate which perception producer or scheduler caused the gap.

The perception guard's freshness evaluation tick gap at this fault was
24.2 ms. The watchdog continued publishing fresh base heartbeats. The run's
base-watchdog input callback maxima were 6.28 ms (trajectory) and 0.28 ms
(steering); its last reported watchdog tick interval was 21.2 ms. This is a
single observation after deferring the log; it does not demonstrate a bound.
The integrated simulator steering-step telemetry covered 3,682 valid steps and
reported maximum internal rate 0.5900000000000012 rad/s against 0.59, with no
violations at 1e-6 tolerance. The separate synchronized SteeringReport/
odometry estimator reported 0.60611 rad/s and failed its 0.6002 observation
criterion. Internal model compliance therefore passed; the downstream
timestamp-pair estimator remains unresolved and is not represented as a
physical steering-rate measurement. Because navigation freshness failed and
the output-rate check failed, the route is a failed validation attempt, not a
passed one-lap result. The separate earlier recorder smoke is also retained
with its missing instrumented-simulator limitation.

This run did not reproduce a clock step. It shows the current limiting failure
is intermittent upstream predicted-object update freshness, while the prior
five-lap failure was simultaneous NDT and trajectory source staleness. The
change to move fault formatting/logging away from the watchdog state lock has
passed focused tests and one live graph exercised the changed watchdog; the
remaining latches show it is not a correction for these upstream data-age
failures.

## Predicted-object timing investigation — 2026-11-13

**Outcome: perception freshness remains intermittent; acceptance is open.**
All new captures are retained in
`/home/marian/roam/validation_evidence/autoware_ackermann/perception_trace_20261113/`.
They used isolated ROS domains and headless, one-route bounded runs. They do
not replace the earlier navigation/fault acceptance evidence.

### Active source-to-watchdog graph

```text
scene.py (10 Hz synthetic PointCloud2, sensor_msgs/PointCloud2)
  /atlas_loc/lidar/raw
  -> atlas_sensor_kit::LivoxToExtendedComponent
  /atlas_loc/lidar/extended
  -> Autoware DistortionCorrectorComponent
  /atlas_loc/lidar/deskewed
  -> atlas_sensor_kit::ExtendedToCompactComponent
  /atlas_loc/lidar/compact
  -> Autoware GroundFilterComponent
  /atlas_loc/obstacles/pointcloud
  -> Autoware EuclideanClusterNode
  /atlas_loc/obstacles/detected_objects (DetectedObjects)
  /atlas_loc/clusters (colored PointCloud2 membership companion)
  -> static_objects.py (exact-stamp membership match + TF at source stamp)
  /atlas_loc/obstacles/predicted_objects (PredictedObjects)
  -> perception_guard.py (300 ms source/receipt freshness checks)
```

There is no tracker or extra detected-to-predicted conversion stage in this
active graph. `scene.py` stamps each synthetic cloud from ROS time at scan
generation (the simulated acquisition time) and publishes on a 100 ms timer.
The timestamp is not rewritten to publication time. The installed point-cloud
conversion/preprocessing chain was observed to preserve exact header stamps;
the Euclidean clusterer builds both output headers from its input header.
`static_objects.py` copies the detection header to the prediction and changes
only `frame_id` to `map`; its TF lookup uses the original message stamp. The
guard therefore checks the original synthetic scan time. A late result remains
late and is rejected by the unchanged 300 ms limit.

The scene publisher uses ROS sensor-data QoS (best effort, keep-last depth 5)
at 10 Hz. The Euclidean cluster input is sensor-data QoS keep-last depth 1;
its `DetectedObjects` and colored membership publishers use reliable depth 1
(`rclcpp::QoS{1}`). The adapter's detection input is reliable depth 10. Its
membership-cloud subscription was best effort, depth 5. The adapter now uses
reliable, volatile, depth 1 for this exact-stamp companion stream. The latest
pair is preferred, and no frame can be paired by approximate time. The adapter
uses a single-threaded `rclpy.spin()` executor and a 10 ms TF retry timer; the
pairing, TF, and bounds conversion normally take a few milliseconds. The
perception components were also tested in a capped two-thread
`component_container_mt`; that experiment did not eliminate the remaining
intermittent age failures, so it is not treated as the root-cause correction.
The launch retains the existing container configuration.

### Measurements and diagnosis

The first instrumented trace (`attempt1`) ended after about 79 seconds despite
the requested 180 second bound. The ROS context stopped and the validator
labeled it `timeout`; it did not reach the deadline, and it is excluded from
route acceptance. It had no freshness latch and predicted-object source age
peaked at 156 ms.

The second trace (`attempt2`, original single-threaded composition) latched on
`predicted_objects`. At the guard fault, source age was 342.4 ms and receipt
age 316.4 ms. Raw and filtered-cloud source ages were 141.7 ms; their receipt
ages were 132.4 and 113.0 ms. The last predicted source stamp was
1794591985.8177254, last received fresh at about 18 ms source age, then aged
past the limit while no newer prediction reached the guard. Exact-stamp source
correlation showed a 195 ms receipt delay between the filtered cloud and the
cluster stage for one preceding frame. The synthetic scene timer itself kept
approximately 100 ms callback cadence, so a slow synthetic scan generator is
not supported by this run.

The third exploratory capture (`attempt5`, with the two-thread component
executor but before the membership QoS change) showed a more specific adapter
pairing symptom: detection stamp 1794593185.8721483 entered the adapter's
pending-membership set. The validator observed a colored cluster cloud with
that stamp, but no adapter-side paired-processing event was captured for the
same stamp before shutdown; the separate predicted-object observer later saw
a prediction with that stamp. This does not prove that the cloud was lost; the
adapter's diagnostic event stream itself can be delayed or incomplete. At the
watchdog fault, predicted source age was 387.6 ms and receipt age 345.7 ms; raw
and filtered-cloud source ages were 247.7 ms. The cluster-stage observer
recorded 440.8 ms source age for a later frame versus 119.9 ms at the
filtered-cloud stage. The active publisher is reliable while the adapter
subscriber was best effort, so the QoS mismatch could lose the required
membership companion while retaining its reliable detection. The adapter
subscription was changed to reliable depth 1 as targeted loss-prevention
hardening; this capture did not establish it as the latch's cause.

Three one-route attempts were then run with the reliable membership QoS
correction (and the two-thread executor comparison enabled):

| Capture | Route result | Freshness result | Relevant measurements |
|---|---|---|---|
| `attempt6` | Goal reached and held; 0.036 m endpoint error | No watchdog latch | Predicted source-age maximum 184 ms; raw 140 ms; NDT 140 ms. Internal steering integration check passed; message-paired rate check failed. |
| `attempt7` | Goal reached and held; 0.046 m endpoint error | No watchdog latch | Predicted source-age maximum 94.6 ms; raw 48.8 ms; NDT 93.7 ms. Internal steering check passed; message-paired rate check failed. |
| `attempt8` | Route interrupted after one loop projection; not at terminal goal | Safe latch on `predicted_objects` and `base_status` | Predicted source age 490.6 ms, receipt age 301.3 ms. Raw and filtered-cloud source ages 284.4 ms, receipt ages 181.3/181.7 ms. Watchdog tick gap 200.6 ms. Run-wide raw receipt gap reached 555 ms and NDT receipt gap 767 ms. |

The QoS change removes the best-effort mismatch for the exact geometry
companion; the captures do not prove that this change caused the successful
runs. It does not resolve the broader interruption: attempt8 latched despite
the adapter publishing the same number of predictions as received detections,
with base status stale at the same evaluation and large raw/NDT delivery gaps.
No one process's compute callback was shown to consume the entire delay.
Euclidean clustering's reported processing-time maximum was
53.8 ms in attempt5; its measured pipeline-latency maximum was 126.3 ms, below
the 628 ms source age later observed by the collector. This leaves additional
executor, DDS delivery, and observation-scheduling delay unlocalized. The
adapter's own detected-to-prediction time had p95/max of 10.5/26.2 ms in
attempt2, 21.2/79.2 ms in attempt5, and 5.2/14.3 ms in attempt8; it is not the
dominant measured delay in those captures. The
validator is a separate Python process with several PointCloud2 observers, so
its callback receipt times can lag; guard-side embedded monotonic diagnostics
and header ages are used for fault conclusions instead of assuming collector
receipt equals publication time.

Resource samples do not establish a single resource cause. Attempt5/8 had
about 1.15–1.28 GiB `MemAvailable`, no memory-full PSI, 1-minute load maxima
3.73/4.55 on six vCPUs, and CPU `some` PSI peaks near 15%. The earlier attempt2
reported only about 80 MiB available in scene-side samples and load near 4.9;
those measurements are consistent with pressure but do not prove it caused
that fault. Since stale output also occurred with over 1 GiB available, low
memory is not a sufficient explanation. CPU saturation, a specific DDS fault,
and a specific executor starvation point are not confirmed.

### Code change and validation status

`static_objects.py` now subscribes to the reliable cluster membership publisher
using reliable depth 1. It still requires exact source-stamp equality, performs
the same timestamped TF lookup, and publishes the unchanged source timestamp.
No freshness threshold, watchdog rule, adapter rejection, or Ackermann control
setting changed. The full-stack launch executor change was exploratory and is
not retained. Focused tests passed (17/17), and `atlas_full_stack_demo` built
sequentially. Two of three post-QoS bounded route runs reached the terminal
goal without a freshness latch; the third safely latched. Therefore this is a
targeted loss-prevention correction, not a resolution of intermittent
predicted-object freshness, and no five-route or five-lap campaign is
justified yet. The broader acceptance status remains **navigation validated;
timing acceptance open**.

The next useful experiment is a low-overhead publisher-side/receiver-side
monotonic trace for the EuclideanCluster `DetectedObjects` and debug-cloud
publish calls, the adapter callbacks, and guard callbacks, including DDS
matched/lost sample counters. It must run without the multiple full-cloud
validator observers. Compare those times to source stamps on a native Ubuntu
host and this VM; do not infer hardware timing from the VM result.

## Existing Humble interfaces

The pinned/installed Autoware distribution is 1.9.0 on ROS 2 Humble. Its
`autoware_control_msgs/msg/Control` message is Ackermann-native:

- `lateral.steering_tire_angle`: tire angle in radians, positive to the left.
- `lateral.steering_tire_rotation_rate`: optional rad/s field with an explicit
  `is_defined_steering_tire_rotation_rate` flag.
- `longitudinal.velocity`: m/s.
- `longitudinal.acceleration`: m/s².

The MPC lateral and PID longitudinal controllers publish this message. The
Autoware vehicle command gate consumes it and publishes the gated `Control` to
the Simple Planning Simulator's `input/ackermann_control_command`. The message
contract already matches; no twist/angular-velocity conversion relay is
required. The simulator applies the bicycle relationship
`yaw_rate = velocity * tan(steering_angle) / wheelbase`.

The existing chain is:

```text
lanelet route and synthetic route scene
  -> Autoware trajectory generation / obstacle-stop planning
  -> Analytical velocity smoother
  -> Autoware trajectory_follower (MPC + PID), using EKF odometry
  -> Autoware vehicle_cmd_gate
  -> Simple Planning Simulator (Ackermann input)
  -> simulator odometry and SteeringReport
  -> synthetic sensors -> NDT -> EKF -> controller feedback
```

The controller consumes `/atlas_loc/ekf/ekf_odom`; simulator truth is not
provided to the controller. The hidden simulator pose is used by the scene to
form idealized simulated measurements and by the validator as evaluation truth
only. The campus Lanelet map provides route geometry. NDT uses a separate
synthetic cloud/map in those coordinates; this does not validate registration
against the actual campus point-cloud map.

## Vehicle parameters

`src/atlas_vehicle_interface/config/ackermann_vehicle.param.yaml` is the shared
provisional geometry source loaded by the Atlas Autoware launches. These values
are simulation placeholders, not measured Atlas specifications:

| Parameter | Simulation value |
|---|---:|
| Wheelbase | 1.20 m |
| Maximum tire steering angle | 0.50 rad |
| Speed limit | 0.80 m/s |
| Simulator acceleration slew | 0.80 m/s² |
| Maximum shared command steering rate | 0.60 rad/s |
| Simulator internal steering-rate cap | 0.59 rad/s |

The Autoware gate retains its more conservative configured longitudinal
acceleration/jerk profile. The localization/full-stack simulator now selects
`DELAY_STEER_ACC_GEARED`; `IDEAL_STEER_ACC_GEARED` instantly applies steering
and ignores `steer_rate_lim`. The selected rate-limited model uses zero command
transport delay and the model's 30 ms minimum time constants. The internal
0.59 rad/s cap leaves margin below the unchanged 0.60 rad/s command limit for
integration and timestamp quantization. These are explicit simulation
assumptions, not physical actuator calibration. The
existing Nav2 simulator, controller, safety gate, and validated configuration
were not changed.

## Launch and validation

Build the Atlas integration packages into a separate overlay (replace the
workspace path if cloned elsewhere):

```bash
source /opt/ros/humble/setup.bash
source <existing-autoware-dependency-overlay>/install/setup.bash
cd <workspace>
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --base-paths src/atlas_vehicle_interface src/atlas_closed_loop_demo \
    src/atlas_localization_closed_loop_demo src/atlas_full_stack_demo \
    src/atlas_trajectory_generation_demo src/atlas_obstacle_stopping_demo \
    src/atlas_perception_planning_demo \
  --packages-select atlas_vehicle_interface atlas_closed_loop_demo \
    atlas_localization_closed_loop_demo atlas_full_stack_demo \
    atlas_trajectory_generation_demo atlas_obstacle_stopping_demo \
    atlas_perception_planning_demo \
  --executor sequential --build-base <overlay>/build \
  --install-base <overlay>/install --cmake-args -DBUILD_TESTING=ON
source <overlay>/install/setup.bash
```

The focused localization/control loop can be launched with:

```bash
ros2 launch atlas_localization_closed_loop_demo localization_loop.launch.py
```

For a bounded full synthetic route run, use an isolated ROS domain and retain
the output outside the source tree:

```bash
export ROS_DOMAIN_ID=113  # choose a currently unused domain
python3 src/atlas_full_stack_demo/scripts/run_lap_validation.py \
  --laps 1 --timeout 180 \
  --output "$HOME/roam/validation_evidence/autoware_ackermann/<run>/lap1"
```

Focused tests include:

```bash
python3 src/atlas_vehicle_interface/test/test_ackermann_model_contract.py
python3 src/atlas_full_stack_demo/scripts/test_lap_metrics.py
python3 -m py_compile src/atlas_full_stack_demo/scripts/run_lap_validation.py
```

## Measurement and current evidence

The full-stack recorder collects simulator truth odometry and steering reports
from the same simulator timer callback. Autoware stamps each output topic
separately after publish work, so the time delta between adjacent
`SteeringReport` stamps can be much shorter than the state-update interval. In
the rate-limited route capture, a direct message-stamp secant reached 2.646
rad/s over a 0.584 ms interval. This was an observation-time artifact, not the
simulator update interval.

The updated evaluator pairs every steering report with its nearest simulator
odometry update stamp (10 ms maximum pairing skew), requires at least 99%
coverage, and computes angle deltas over those update intervals. The preserved
rate-limited lap capture yielded 5,473/5,475 paired samples, maximum pairing
skew 8.37 ms, and maximum synchronized update-to-update steering rate 0.600104
rad/s. The 0.0002 rad/s tolerance is only for float/time quantization. A
regression test exercises this timestamp pairing. This is a per-update average
rate measurement; it is not a hardware actuator characterization.

Evidence is stored outside the repository under
`~/roam/validation_evidence/autoware_ackermann/`. Relevant captures include:

- `control_aligned_*`: straight control loop on aligned speed/steering limits.
- `full_route_20261112T012929Z/lap1`: failed acceptance with the ideal model
  and smoother override; goal reached, but raw speed and reported steering
  rate checks failed.
- `full_route_rate_limited_20261112T013356Z/lap1`: route completed and held
  with rate-limited dynamics and 0.8 m/s smoother limit; original raw
  cross-topic derivative failed, while synchronized replay passes.
- `full_route_059_20261112T014443Z/lap1`: final bounded full-stack route with
  synchronized rate evaluation and a 0.59 rad/s simulator-side steering-rate
  cap. All 12 acceptance checks passed. Goal error was 0.0208 m, maximum
  controller-reference cross-track error 0.1729 m, maximum heading error
  0.1209 rad, maximum speed 0.8000 m/s, maximum steering angle 0.3275 rad,
  maximum synchronized steering rate 0.5906 rad/s, maximum measured
  acceleration 0.2513 m/s², and no safety latch was observed.
- `command_loss_20261112T014946Z/run_20261112_014946_drop_command_IDEAL_STEER_ACC_GEARED_gated`:
  control-loop command loss passed all harness checks. At 0.5023 m/s the gate
  detected the lost command in 0.3103 s, stopped in 1.7021 s over 0.5978 m,
  held zero speed/displacement for the final three seconds, and refused rearm
  while inputs were invalid. Valid inputs were restored without rearming.
- `stale_localization_20261112T015032Z/run_20261112_015032_freeze_odometry_IDEAL_STEER_ACC_GEARED_gated`:
  frozen odometry passed all harness checks. At 0.5014 m/s the gate detected
  stale odometry in 0.3078 s, stopped in 1.6823 s over 0.5847 m, held still,
  and refused rearm after valid data was restored. These two fault runs use
  the control-only harness and ideal geared simulator; they validate the
  existing gate/watchdog response, not rate-limited full-stack dynamics.

### Diagnostic recorder overhead and timing conclusions — 2026-11-13

The full-stack evaluator previously attached best-effort depth-one observers
to raw and five intermediate `PointCloud2` topics on every run. The callbacks
retained only source stamp, item count, frame, ROS age, and local monotonic
receipt time, but ROS still deserialized each full cloud in the evaluator
process. The raw-cloud observer remains enabled because it is the sensor-age
baseline; the five intermediate-cloud observers are now opt-in with
`--trace-perception`. Default route acceptance continues to collect raw source
and receipt age, scalar detections/predictions, adapter timing, cluster debug
metrics, guard/watchdog status, NDT/EKF/trajectory ages, scene timing, and
process scheduling samples without those five additional full-cloud
subscriptions. `test_perception_trace_configuration.py` verifies the optional
intermediate observer set is empty by default and contains each intended stage
exactly once when enabled. Existing traces are retained and were collected
with the earlier six-observer configuration; this code change has not yet been
runtime-compared.

The existing captures support these conclusions, with uncertainty kept
explicit:

- **Confirmed:** at freshness latches, predicted-object headers retained old
  source time and exceeded the unchanged 300 ms bound. Guard receipt age could
  be lower or higher, depending on the event. The adapter preserves the
  DetectedObjects stamp and normally measured a short callback-to-publish time.
- **Measured but not publisher-localized:** the scene published scans at about
  10 Hz and its callback timing did not show an equivalent long generation
  callback in the captured events. Stage observer source-age differences
  sometimes rose between ground-filter output and cluster output; cluster
  debug metrics were shorter than the worst observed source-age peaks. The
  observations do not show whether the cluster input/output queue, executor,
  DDS delivery, or evaluator scheduling accounts for the remainder.
- **Not confirmed:** the reliable depth-one membership-cloud subscription
  removes a best-effort/reliable mismatch for its exact-stamp companion. It is
  sensible loss-prevention hardening, but later predicted-object freshness
  failure shows it is not sufficient to explain or eliminate the broader
  interruptions. The two-thread composition experiment also did not establish
  a correction and is not retained in the launch.
- **Localization:** the recorder captures NDT and EKF source stamps and
  monotonic receipt times, NDT execution-time debug values, trajectory stamps,
  watchdog tick/callback timing, runtime queue warnings, and simulator truth
  only as evaluation data. Previous NDT/trajectory faults had similar old
  source stamps and short NDT compute time; this points to delayed data flow but
  does not identify the delayed producer. EKF twist-queue overflow indicates
  queue pressure/drop behavior, not localization accuracy. Synthetic sensor
  inputs and the synthetic NDT map do not validate campus map registration.

Before the later bounded Mac smoke documented below, the latest full-stack
outcomes were the two goal-held traces and one safe latch in
`perception_trace_20261113`. The optional trace flag enables detailed
diagnosis separately from lower-overhead acceptance runs, but does not itself
fix freshness. The remaining publisher-side evidence is best collected from
an isolated instrumented Euclidean-cluster build on native Ubuntu, correlating
exact source headers with component monotonic input, callback-start/end and
publish times and DDS matched/lost counters. Do not add more full-cloud
observers to the acceptance path.

### Steering-rate timestamp reconciliation

The simulator's instrumented steering diagnostic is emitted inside its
integration callback and records actual `dt`, previous angle, requested angle,
applied angle, increment, computed internal rate, and configured cap. The
corresponding simulation captures report a maximum internal rate of
approximately 0.59 rad/s with no integration-step violations within the
documented 1e-6 rad/s floating-point tolerance. This is the authoritative
measurement for the simulator's configured rate limiter.

The downstream `SteeringReport` receives a `builtin_interfaces/Time` stamp at
report publication; odometry is stamped at its publication in a separate call
after the integration step. Their headers do not share an integration-step
identifier. Nearest-stamp pairing reduces error but cannot prove exact
angle-to-step correspondence, and irregular spacing makes message-to-message
differentiation sensitive to pairing. This is the strongest supported
explanation for small downstream over-limit secants such as 0.5906 rad/s, but
it is not proven without a common step sequence or timestamp in both output
messages. Limits were not raised. Keep reporting raw paired measurements for
transparency and use internal step telemetry for simulation-model compliance;
neither establishes physical actuator behavior. The acceptance validator now
uses the internal integration-step rate as the steering-limit criterion only
when it has more than 100 valid steps and at least 99% coverage against
simulator odometry. Missing telemetry fails closed. The paired report/odometry
estimate remains in the result as an explicitly informational diagnostic.
Regression tests verify that a compliant internal trace passes, an actual
over-limit model step fails, and missing telemetry fails.

### Native migration entry point

See [`NATIVE_UBUNTU_VALIDATION.md`](NATIVE_UBUNTU_VALIDATION.md) and
`tools/autoware/run_ackermann_native_validation.sh`. The runner has non-mutating
preflight, targeted sequential build, one-route smoke, current full-graph
E-stop case, five separate one-lap runs, and a distinct continuous five-lap
run. The native campaign was not executed on this ARM64 VM. The current
full-graph safety harness does not yet cover all requested fault classes;
do not treat its E-stop case or localization-demo fault tests as full matrix
acceptance.

The EKF reported a twist queue of 3 against its configured maximum of 2; the
synthetic scene supplies twist derived from simulator feedback, so this is an
idealized localization input rather than independent localization evidence.
The map loader warning came from a synthetic fixture payload whose point count
does not match the campus PCD. NDT still consumes synthetic loop-scene scans
and provides feedback, but campus point-cloud registration is not validated.

## Acceptance campaign — 2026-11-12

Evidence is retained under
`~/roam/validation_evidence/autoware_ackermann/acceptance_20261112T0310/`.
The six sequential one-lap captures (`run1` through `run6`) produced:

| Attempt | Result | Progress / key observation |
|---|---|---|
| run1 | Route complete and held | 1 lap; terminal error 0.0224 m; no latch |
| run2 | Safe watchdog latch | 0.078 lap; multiple input streams became stale |
| run3 | Route complete and held | 1 lap; terminal error 0.0583 m; no latch |
| run4 | Safe watchdog latch | 0.124 lap; NDT source age reached about 0.345 s while receipt age was about 0.130 s |
| run5 | Route complete and held | 1 lap; terminal error 0.0447 m; no latch |
| run6 | Safe watchdog latch | 0.133 lap; predicted-object source age 0.310 s, receipt age 0.162 s |

These runs establish successful route following in individual trials, not
repeatability acceptance. No five-lap run was performed. The route-wide
projection metric has a discontinuity at the loop seam (RMS near 3.1 m and
maximum near 10.4 m on successful laps); controller-reference cross-track
metrics remained around 0.06 m RMS and 0.12 m maximum. Do not use the seam
metric as controller tracking error.

The first earlier campaign run contained a ROS time discontinuity: ROS source
time advanced about 1172.9 s over 0.0218 s monotonic receipt time, followed by
an implausible steering state and a safe watchdog latch. A system log recorded
a clock-change event and NTP was active. This strongly supports a guest clock
step as the proximate cause; whether it came from NTP correction or VM time
synchronization is unresolved. Later runs had no comparable clock jump. Other
safe latches arose from stale NDT and predicted-object source timestamps. The
evidence does not establish one common root cause or prove CPU saturation.

### E-stop clear and route resumption

The initial full-graph E-stop clear test showed a real behavior defect: after
the gate's emergency condition was explicitly cleared, the still-active route
command became authorized again. The simulation-only
`estop_disengage.py` safeguard now sends `Engage(false)` during the latched
E-stop and does not issue `Engage(true)`. In the retained retest
`estop_clear_retest/result.json`, E-stop at 0.3192 m/s produced zero gate output
in 26.5 ms, emergency status in 26.6 ms, simulator deceleration began in
105.6 ms, and standstill occurred after 2.725 s and 0.1351 m. The documented
simulation envelope was 0.2977 m. Explicit clear was accepted; the gate remained
disengaged, its output stayed at zero, and the previous route command did not
resume. A new explicit engage/navigation action is required. A prior failed
capture is retained at `estop_clear/result.json`.

This is software-path evidence only. The gate and simulator remain dependent on
the same guest OS scheduler. A paused process cannot run its own watchdog or
deliver a stop command. Physical deployment requires an actuator-side command
timeout and an appropriately independent hardware E-stop design; neither has
been implemented or validated here.

### Steering-rate interpretation

The simulator model clamps steering derivative to its configured 0.59 rad/s
limit at its integration step. Run captures report nearest-odometry-stamp
steering secants of 0.59045, 0.59408, and up to about 0.59155 rad/s on the
successful and failed route trials. These are not direct measurements of the
internal integration derivative: odometry and SteeringReport timestamps are
stamped separately after the integration update, and sample intervals are
irregular. The sampled values exceed the configured cap by up to about
0.0041 rad/s; no integration-step telemetry currently explains the difference.
Therefore the integration-step limit is implemented in the pinned simulator,
but runtime rate verification against the 0.59 cap remains inconclusive. The
configured limit was not increased.

### Localization and map evidence

The full-stack controller consumes `/atlas_loc/ekf/ekf_odom`; no direct
simulator-truth subscription to the controller was found. The synthetic scene
uses simulator state to synthesize sensor and twist inputs, while the validator
uses truth only as evaluation reference. EKF twist-queue overflow warnings
indicate backlog/drop behavior (configured maximum queue is 2), not proof that
localization is accurate. Run4's NDT execution duration was about 19.8 ms while
its source data was already old at receipt, indicating delayed source/data flow
rather than a demonstrated long NDT callback. Run6 likewise had fresh raw
cloud receipts but delayed predicted-object source stamps. The `map_routing_probe`
warning was traced to a synthetic point-cloud fixture/map count mismatch; the
development map-integration package was rebuilt into the active overlay and
the subsequent route logs no longer emitted that warning. This does not
validate registration against the real campus PCD.

### Acceptance matrix

| Requirement | Evidence / attempts | Status |
|---|---|---|
| Ackermann MPC/PID route following | One-lap route completed in runs 1, 3, 5; 3/6 campaign attempts completed | Implemented; individual runs validated; repeatability incomplete |
| Curved-route repeatability | Same route includes curves; only 3 successful attempts, not five consecutive | Incomplete |
| Five full route laps | No five-lap campaign in this acceptance sprint | Not run |
| Steering-angle constraint | All six captures stayed below 0.50 rad (peak about 0.330 rad) | Passed in simulation captures |
| Steering-rate constraint | Model has 0.59 cap; sampled estimates 0.590–0.592+ rad/s, integration-step output unavailable | Inconclusive |
| Acceleration constraint | Per-run odometry-derived peaks below 0.80 m/s² in available captures | Observed within configured limit; simulator-step proof not added here |
| Full-graph E-stop and explicit rearm | E-stop/clear retest passed, including no old-route resumption | Passed for this tested path |
| Full-graph command loss, odometry loss, invalid command, sensor loss | Existing control-only tests cover command loss/frozen odometry; full-graph versions not run | Incomplete |
| NDT/campus localization accuracy | Synthetic NDT/EKF feedback only; invalid synthetic map payload warning corrected in overlay | Not validated against campus map or physical sensors |
| Hardware safety/readiness | No physical actuator, timeout, E-stop, or vehicle test | Not tested |

The result is **Autoware Ackermann simulation partially validated**. It is not
eligible for a completed acceptance designation because route repeatability,
five-lap behavior, broad full-graph fault injection, and internal steering-rate
measurement remain outstanding.

### Regression snapshot

On 2026-11-12, the eight changed Autoware integration packages
(`atlas_map_integration_demo`, `atlas_vehicle_interface`,
`atlas_closed_loop_demo`, `atlas_trajectory_generation_demo`,
`atlas_localization_closed_loop_demo`, `atlas_obstacle_stopping_demo`,
`atlas_perception_planning_demo`, and `atlas_full_stack_demo`) built
sequentially with `CMAKE_BUILD_PARALLEL_LEVEL=1` into the isolated
`atlas_autoware_ackermann_v2` overlay. The Nav2 package built in a separate
overlay and its `colcon test` target passed: 64 pytest cases plus 4 C++ gtests.
The focused E-stop, perception-delivery, and lap-metric Python suites passed
19 tests; the shared vehicle-parameter/message-contract test passed. These are
software regression results, not route acceptance or hardware validation.

## Interface and safety boundary

`atlas_vehicle_interface` currently documents the future vehicle-driver
boundary and shared model parameters. It does not communicate with physical
actuators. Before hardware work, Atlas geometry, steering rack conversion,
steering and brake dynamics, encoder scale, and command timeout must be measured
and specified. The actuator endpoint must enforce command timeout and fail-safe
zero/stop behavior independently; software simulation results do not establish
hardware safety.

Keep Nav2 and Autoware mutually exclusive as command owners. The simulator is a
single endpoint in each launch graph; do not launch both command stacks against
the same simulator.

## Bounded Mac migration smoke — 2026-11-13

The initial short smoke used a fresh overlay containing only
`atlas_full_stack_demo` over the older timing-fix overlay. It resolved an older
`atlas_map_integration_demo`, logged an invalid synthetic point-cloud payload
and missing transforms, then safely latched on stale NDT and trajectory at
0.098 lap. This run is retained as a setup failure, not algorithm acceptance.

I rebuilt both `atlas_map_integration_demo` and `atlas_full_stack_demo` into
`/home/marian/roam/diagnostic_overlays/atlas_mac_final_20261113`. Prefix checks
show both resolve from that overlay. The corrected overlay completed and held
the one-lap goal: 76.44 m traveled, terminal position error 0.0428 m, maximum
controller-reference cross-track 0.1177 m, maximum synchronized report rate
0.59025 rad/s, and acceleration peak 0.2490 m/s². However, the active launch
did not enable simulator-step telemetry, so the required internal-rate check
had zero samples and the overall validator correctly returned `passed=false`.
The paired output-rate estimate cannot substitute for the missing integration
telemetry.

The recorder now exposes `--steering-step-diagnostics` and passes the launch
argument explicitly. A final bounded trial used the instrumented simulator and
corrected map overlay. Internal step telemetry passed: 177/177 valid steps,
100% coverage against simulator odometry, maximum internal rate
0.5900000000000001 rad/s versus the configured 0.59 cap, zero violations at
1e-6 tolerance. That trial safely latched almost immediately (0.00158 lap,
0.0208 m traveled): NDT source age was 316.9 ms while receipt age was 143.7 ms;
trajectory source age was 287.6 ms and receipt age 144.7 ms. Raw cloud and
predicted objects carried the same stale source stamp (313.7 ms) although
their receipt ages were 149.6 ms and 140.5 ms. This demonstrates source-data
age accumulation before/at the observed stages, not delayed guard evaluation;
the watchdog tick gap was 22.0 ms. No individual C++ publisher/DDS/executor
cause is identified.

The scene-side event sequence provides the first direct upstream observation
in these captures: at monotonic 92570.201349 s the scene published a scan
whose preserved source header was 30.75 ms old. Its synchronous route-message
publish call took 29.31 ms within a 31.70 ms scene callback. At monotonic
92570.335754 s the next 100 ms timer ran 134.40 ms after the prior tick
(34.40 ms scheduling delay); the cached simulator odometry header was already
110.31 ms old, so the scene correctly skipped that scan under its unchanged
100 ms sensor-input rule. Only one odometry callback had arrived since the
prior scene tick. Predicted objects/NDT later crossed their 300 ms source-age
limits while the guard itself was ticking at 20 ms cadence. Thus the immediate
mechanism is a missing scan caused by stale simulator feedback, followed by
aging of the previous perception/localization result. The 29 ms route publish
blocks the scene's single-threaded executor and is a plausible local contributor
to callback contention, but the trace does not prove it caused the 34 ms timer
lateness or sparse simulator odometry delivery.

Resource telemetry at the same time is mixed: system load was about 2.26 on six
vCPUs; `/proc` process sampling showed roughly 1.17 GiB `MemAvailable`, no
memory PSI, CPU PSI `some` near 11%, and aggregate per-process run-queue wait
around 0.3–0.47 s over the final 1.13 s sample. This is evidence of scheduling
pressure, not CPU saturation and not proof of why the simulator/scene callbacks
were delayed. The scene's own historical field named
`system_available_memory_bytes` used `SC_AVPHYS_PAGES` (free pages, not Linux
`MemAvailable`); the sampler now reads `/proc/meminfo` and reports available
and free bytes separately. The old captured value is therefore not used as
available-memory evidence.

The paired report/odometry method in that short run peaked at 0.60686 rad/s
(maximum pair skew 8.54 ms), while internal integration telemetry passed.
Since message timestamps are assigned at separate output publications after
the same model step, the discrepancy remains unresolved without a common step
identifier. Do not relax the gate observation criterion or treat the one
internally compliant simulator run as physical steering evidence.

All three captures are under
`~/roam/validation_evidence/autoware_ackermann/mac_sprint_20261113T/`:
`full_route_smoke`, `corrected_full_route_smoke`, and
`instrumented_full_route_smoke`. Only the middle run reached and held the goal,
but its overall acceptance failed for missing integration telemetry; the
instrumented run failed navigation due to a real freshness latch. No full-graph
fault injection was attempted after the repeated unexpected freshness latches.
The new code/tests passed the 145-case focused regression, but repeatability
and timing acceptance remain open.
