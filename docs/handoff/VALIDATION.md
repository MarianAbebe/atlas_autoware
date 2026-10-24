# Validation status at handoff

## Latest synchronized stationary run

- Bag: `/tmp/atlas_stability_20261023_213504/timing_bag`
- Scheduler telemetry: `/tmp/atlas_stability_20261023_213504/scheduler_telemetry`
- Bag interval: `1792791308.662609979` through approximately `1792791499.233472`
- Duration: 190.570861695 seconds
- Localization bootstrap: `1792791392.220018794`
- Simulator speed and displacement after bootstrap: exactly zero in the recording

### Recorded counts

| Topic | Messages |
|---|---:|
| `/diagnostics` | 21,837 |
| `/atlas_loc/sim/output/odometry` | 9,415 |
| `/atlas_loc/base_safety_status` | 9,498 |
| `/atlas_loc/safety_status` | 9,497 |
| `/atlas_loc/base_safety_heartbeat` | 5,235 |
| `/atlas_loc/ekf/ekf_odom` | 5,350 |
| `/atlas_loc/control_cmd` | 3,524 |
| `/atlas_loc/raw_trajectory` | 1,899 |
| `/atlas_loc/obstacle_trajectory` | 1,059 |
| `/atlas_loc/trajectory` | 1,059 |
| `/atlas_loc/diagnostics/scene_timing` | 1,899 |
| `/atlas_loc/ndt/ndt_pose_with_covariance` | 1,068 |
| `/atlas_loc/ndt/exe_time_ms` | 1,068 |
| Deskew latency diagnostic | 1,877 |
| Clustering latency diagnostic | 1,877 |

### Watchdogs

- No latch occurred in any of 9,498 base or 9,497 combined status messages.
- Base armed at source time `1792791394.5393324`.
- Combined armed at source time `1792791395.5688505`.
- Neither became unarmed afterward.
- Startup stale inputs cleared before arming; no stale input was reported after
  arming.
- Maximum base timer/source gap: 37.503 ms.
- Maximum combined timer/source gap: 34.739 ms.
- Final recorded statuses at `1792791499.2189865` and `1792791499.229630` were
  armed, unlatched and healthy.

### Timing and processing

| Measurement | Median | p95 | p99 | Maximum |
|---|---:|---:|---:|---:|
| NDT execution | 14.431 ms | 19.175 ms | 31.346 ms | 93.203 ms |
| Deskew pipeline | 30.418 ms | 34.757 ms | 39.837 ms | 85.615 ms |
| Clustering pipeline | 46.701 ms | 55.566 ms | 68.630 ms | 104.213 ms |
| Scene callback duration | 11.377 ms | 15.006 ms | 20.716 ms | 60.974 ms |

Scene callback gaps never exceeded 106.180 ms, cached odometry age never exceeded
24.427 ms, and no scan was skipped.

NDT acquisition stamps stayed near 10 Hz, with a maximum source gap of 102.216
ms. One output-delivery interruption reached 206.717 ms and source-to-bag latency
reached 156.590 ms. Bag-order reconstruction estimated a maximum NDT age near a
base-watchdog tick of 253.147 ms. This is a proxy, not the watchdog's internal
receipt timestamp, and leaves approximately 46.9 ms below the safety threshold.

### Scheduler telemetry

Telemetry covered 158.374 seconds, including bootstrap and the full armed period.
The 50 ms sampler had a 49.999 ms median interval and 95.504 ms maximum interval.
System CPU was 72.67% idle on average, but CPU PSI `some` accumulated for 10.657%
of wall time and reached 13.56% avg10. CPU steal was zero. Memory and I/O PSI were
below 0.1% overall.

Available memory stayed between 1,176 and 1,361 MiB. Swap occupancy was about
1.09–1.11 GiB; 249 major faults, approximately 0.78 MiB swap-in and 17.4 MiB
swap-out occurred. No paging or steal occurred during the most important timing
event.

At approximately `1792791403.95`, runnable tasks reached 45 while NDT, both
watchdogs, scene and controller each accumulated 20–43 ms of run-queue wait in
one sample interval. NDT and perception messages then arrived more closely spaced.
This supports a smaller episode of guest-side scheduling/executor contention; it
does not support an NDT algorithm failure.

The telemetry classifier did not capture the rosbag recorder process, so recorder
CPU, wait and RSS remain unknown.

## Previous fault evidence

- Bag: `/tmp/atlas_trajectory_diagnostic_01/timing_bag`
- Combined fault: `1792786676.5425935`
- Base fault: `1792786676.5442383`

The combined timer gap was 331.600 ms and the base fault timer gap was 207.112
ms. Independent processes accumulated messages and delivered them in bursts:

| Stream | Maximum fault-window receipt gap |
|---|---:|
| Simulator odometry | 234.443 ms |
| EKF odometry | 239.273 ms |
| NDT | 296.363 ms |
| Smoothed trajectory | 319.800 ms |
| Base heartbeat | 224.688 ms |

Both watchdogs correctly enforced their unchanged freshness contracts. That run
has no scheduler telemetry, so host versus guest scheduling cannot be separated.

## Other retained full-stack results

The earlier fixed batch under `docs/full_stack/validation` recorded:

- Clear route: 2/2 passed.
- Isolated NDT loss: 2/2 passed.
- Shared LiDAR loss: 2/2 passed.
- Obstacle stop: 1/2 passed; the failure was a perception-freshness latch and
  was not counted as planned stopping.

These historical engaged experiments demonstrate that the connections execute;
they do not establish current reproducibility or physical accuracy.

## Verified versus unverified

Verified on the latest stationary run:

- Localization initialization, NDT and EKF publication.
- LiDAR conversion, deskew, ground filtering and clustering activity.
- Raw, obstacle and smoothed trajectory publication.
- MPC/PID command publication without simulator engagement.
- Continuous base and combined safety authorization for over 103 seconds.
- Safety gate prevented motion without explicit engagement.

Not verified by the latest run:

- Engaged route following or goal completion.
- Planned obstacle braking and hold behavior.
- Localization-loss stopping while moving.
- Localization accuracy against hidden truth.
- Repeatability across machines.
- Real PCD alignment, real sensor behavior or physical vehicle operation.
