# Safe troubleshooting

Always source `src/atlas_full_stack_demo/scripts/environment.sh` first. Never
increase the 300 ms limits, disable a guard, substitute simulator truth, or count
a latch-induced stop as planned behavior.

## Missing TF before localization initializes

`map` to vehicle TF can be absent before EKF/NDT activation and bootstrap. Check
the activation services and sensor input without publishing anything manually:

```bash
ros2 service list | rg '/atlas_loc/(ekf|ndt)/trigger_node_srv'
ros2 topic echo /atlas_loc/scene_info --once
ros2 topic echo /atlas_loc/lidar/compact --once --field header
ros2 topic echo /atlas_loc/sim/output/odometry --once --field twist.twist
```

Then use the bounded helper:

```bash
ros2 run atlas_full_stack_demo initialize_localization.py
```

It refuses moving or latched initialization and does not engage. After success,
inspect `map` to `base_link` with `tf2_echo`. Do not fabricate a transform.

## EKF queue, delay or covariance warnings

Short startup warnings occurred around bootstrap in the latest recording. They
are not acceptable if they persist after both watchdogs arm.

```bash
ros2 topic echo /diagnostics
ros2 topic hz /atlas_loc/ndt/ndt_pose_with_covariance
ros2 topic hz /atlas_loc/ekf/ekf_odom
```

Stop the `hz` commands with Ctrl+C. If NDT is near 10 Hz and EKF near 50 Hz but
warnings persist, preserve `/diagnostics`, NDT execution timing and the launch
log. Do not engage.

## NDT freshness failure

An NDT latch can result from old scan acquisition time, delayed NDT processing,
or callback/message delivery delay. Capture all three rather than treating it as
an NDT convergence failure:

```bash
ros2 service call /atlas_loc/full_stack_scene/dump_scene_timing \
  std_srvs/srv/Trigger '{}'
ros2 topic echo /atlas_loc/base_safety_status --once
ros2 topic echo /atlas_loc/ndt/exe_time_ms --once
```

Preserve the diagnostic bag and scheduler telemetry described below.

## Trajectory freshness failure

Trace each stage separately:

```bash
ros2 topic hz /atlas_loc/raw_trajectory
ros2 topic hz /atlas_loc/obstacle_trajectory
ros2 topic hz /atlas_loc/trajectory
ros2 topic echo /atlas_loc/base_safety_status --once
```

Normal stationary publication is approximately 10 Hz. A watchdog source-age
failure is not equivalent to an interarrival gap; retain header stamps and bag
receipt times.

## Watchdog latch

1. Do not engage, rearm or restart immediately.
2. Record both complete JSON status messages.
3. Dump scene timing.
4. Stop and finalize the diagnostic recorder and telemetry.
5. Copy the launch log and note the Unix fault timestamp.
6. Shut down the launch cleanly.

The stationary initializer deliberately refuses to run while the combined guard
is latched. Explicit rearm exists for controlled development but also requires
fresh inputs and live standstill; it is not a demonstration recovery procedure.

## Scheduler pressure

Evidence of a graph-wide scheduling event includes simultaneous message receipt
gaps, watchdog timer gaps, increasing per-process run-queue wait, a sampler gap,
runnable-task or CPU PSI spikes, and later burst delivery. CPU utilization alone
is insufficient.

Minimize unrelated host applications, builds and RViz. Keep one graph instance.
If a fault occurs, compare `processes.csv` cumulative counter deltas and
`system.csv` pressure deltas around the rosbag realtime timestamp.

## Duplicate ROS graphs

```bash
ros2 node list --no-daemon --spin-time 3 | sort
ros2 component list /atlas_loc/perception_container
ps -eo pid,ppid,etime,stat,comm,args | rg \
  'full_stack.launch.py|perception_container|simulator_container|scene.py'
```

A clean perception container contains exactly `livox_converter`,
`distortion_corrector`, `compact_converter`, `ground_filter`, and
`euclidean_cluster`. Stop duplicate owning launch terminals with Ctrl+C. Do not
delete evidence or kill unrelated processes.

## Diagnostic recorder

Start the recorder before scheduler telemetry because the recorder creates the
parent result directory:

```bash
RUN=/tmp/atlas_stability_$(date -u +%Y%m%d_%H%M%S)
echo "$RUN"
ros2 run atlas_full_stack_demo record_timing_diagnostics.sh "$RUN"
```

It records 15 timing/safety/trajectory topics and intentionally excludes point
clouds. It refuses to overwrite an existing path. Ctrl+C is forwarded to rosbag
for clean finalization.

## Scheduler telemetry

In a second sourced terminal, use the exact `RUN` path printed by the recorder:

```bash
RUN=/tmp/atlas_stability_YYYYMMDD_HHMMSS
ros2 run atlas_full_stack_demo scheduler_telemetry.py \
  --output "$RUN/scheduler_telemetry"
```

`realtime_ns` correlates with rosbag timestamps; `monotonic_ns` is used for
interval and scheduler calculations. Stop telemetry first, then rosbag, then the
launch. The current process matcher did not identify the rosbag recorder in the
latest run; confirm the labels in `processes.csv` before assuming recorder load
was captured.
