# Five-lap engineering progress

Date: 2026-10-31  
Worktree: `/home/marian/roam/atlas_autoware_dev`  
Branch/HEAD: `feature/prd-gap-closure` / `ed82c0a19a77bd06691269d087bbacadaebe4317`

## Baseline and safety

- Development changes are uncommitted and preserved. No stage, commit, or push.
- Stable checkout `/home/marian/roam/atlas_autoware` remains clean on
  `demo/atlas-v1`.
- Existing dependency overlays remain untouched. New package output is under
  `/tmp/atlas_five_lap_geom_cmake_{build,install}_20261031`.
- Preserved full-stack evidence is under `/tmp/atlas_five_lap_1_host_20261031_*`;
  no evidence directory was overwritten.

## Experiments and conclusions

| ID | Hypothesis / action | Result | Next action |
|---|---|---|---|
| prior scene stress | The scene itself cannot sustain 10 Hz scans. | 180 s scene-only test passed: 1,795 scans, 9.96 Hz, max receipt gap 164.5 ms, no stale-input skips. Uses deterministic synthetic odometry; does not reproduce full-stack load. | Retain as scene-only evidence; investigate full-stack input gaps on host. |
| prior Run 09 trace | A synchronous trajectory publish on stale odometry may block the scene executor. | Evidence showed a 316 ms trajectory publish in the stale branch, followed by stale odometry and sensor gaps. The stale branch now skips route publication; focused scene regression covers it. | Verify in a live full-stack run. |
| Run 13 startup | A failed component-load response means the component failed to instantiate. | Not supported: the Livox component instantiated and processed scans, while the service response failed at the RMW layer. Fifteen isolated component-load tests passed; full-stack service response failure remains unresolved. | On host, instrument manager state and service response with fresh startup trials; do not blind-retry loads or authorize without fresh required topics. |
| Run 30 terminal path | The controller overshot despite a zero-speed terminal trajectory. | Trajectory/controller evidence plus Autoware's first-nearest search with a 3 m soft distance limit indicates terminal poses could associate with the initial branch: old geometric separation was 2.821 m. The map now starts at 8 m; geometry regression measures 4.440 m separation. No controller or safety threshold changed. | Validate the geometry in one live lap; correction is provisional until terminal hold passes. |
| Run 30 freshness | Scene timing overrun caused the watchdog latch. | Not established. Scene tick schedule/work at the latch were 98 ms / 0.000039 s, but raw/NDT header gaps reached 0.281 s and NDT receipt gap 0.438 s. Watchdog observed 0.330 s source age. Upstream pause cause remains unknown. | Compare publisher, scene, and perception receipt streams during a host run. |
| valid-domain DDS probe | ROS cross-process communication may now be allowed. | Failed on valid domain 231. Evidence: `/tmp/atlas_ros_cross_process_probe_20261031_01/`; ROS daemon socket permission denied, Fast DDS `getifaddrs` and UDP socket creation denied, no probe message received. Do not retry or bypass sandbox. | Run the bounded host validation below on an authorized Ubuntu terminal. |

## Offline verification

- Focused tests: 33/33 passed (scene 15, map 2, lap metrics 9,
  localization watchdog 7).
- Python compilation and `git diff --check`: passed.
- Focused `atlas_full_stack_demo` CMake configure/build/install, one job:
  passed. Logs: `/tmp/atlas_five_lap_geom_cmake_{configure,build,install}_20261031.log`.
- Installed package resolves to `/tmp/atlas_five_lap_geom_cmake_install_20261031`;
  launch parsing succeeded and installed OSM/PCD match source.
- `colcon` did not complete; direct CMake success is not a colcon result.
- Current package launch args were inspected. Full-stack launch has no standalone
  runtime validation result for the updated geometry.

## Acceptance state

- Startup trials: no new full-stack trial; isolated load tests previously 15/15.
- Sustained full-stack sensor continuity: NOT VERIFIED.
- One-lap terminal-stop acceptance: NOT PASSED.
- Two-lap and five-lap acceptance: NOT RUN.
- RViz motion/screenshots: NOT RUN.
- Hardware validation: NOT PERFORMED.

Run 30 was a failure (1.493 projected passes, 104.951 m truth travel,
watchdog latched). Its localization error was 0.0442 m 3D RMSE and
0.00436 rad yaw RMSE. These are not results for the new geometry.

## Next executable action (host only)

On the authorized Ubuntu host, first choose an unused valid ROS domain in
`0..232` (do not use 235). Then run one headless lap with the updated package
prefix taking precedence:

```bash
source /home/marian/roam/atlas_autoware_dev_env.sh
source /tmp/atlas_five_lap_diag_20261031_d/install/setup.bash
source /tmp/atlas_five_lap_diag_20261031_e/install/setup.bash
export AMENT_PREFIX_PATH=/tmp/atlas_five_lap_geom_cmake_install_20261031:$AMENT_PREFIX_PATH
export ATLAS_AUTOWARE_ROOT=/home/marian/roam/references/autoware
export ROS_DOMAIN_ID=226  # only if confirmed unused on that host
export ROS_LOG_DIR="/tmp/atlas_five_lap_host_logs_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$ROS_LOG_DIR"
python3 /home/marian/roam/atlas_autoware_dev/src/atlas_full_stack_demo/scripts/run_lap_validation.py \
  --laps 1 --timeout 240 \
  --output "/tmp/atlas_five_lap_host_1_$(date -u +%Y%m%dT%H%M%SZ)"
```

Keep the run evidence even on failure. Proceed to two laps only after one lap
passes sensor continuity, perception authorization, terminal-spur traversal,
goal tolerance, stopped hold, and no-watchdog criteria.

## Base watchdog / guard receipt-gap follow-up

Evidence: `/tmp/atlas_five_lap_host_1_20261031T092123Z/` (host run, 1 lap,
26.265 m truth travel, 0.4139 fractional lap, 0 completed laps). It failed
`watchdog_latched`; terminal behavior was not reached. Cross-track RMS was
0.0427 m, heading RMS 0.0340 rad, localization 3D RMSE 0.0840 m.

Confirmed behavior:

- The full-stack outer perception guard first lost freshness of `base_heartbeat` and
  `base_status` at about 0.307 s source/receipt age. At that instant LiDAR,
  point-cloud, and predicted-object source ages were about 0.065 s. Its timer
  still ran at about 21 ms intervals and ~0.22 ms work.
- The localization watchdog separately logged `LATCHED stale required input:
  ndt` at ROS time 1793438527.292. Its heartbeat withholding after that
  latch is the intended safety behavior, not a publisher workaround target.
- Immediately before the guard latch, its last accepted base status/heartbeat
  stamp was about 1793438526.979. Other guard inputs continued updating. Base
  status later resumed, while the heartbeat remained withheld after the base
  localization latch. This localizes an additional gap to base-status/heartbeat
  delivery or callback servicing at the guard; the captured run cannot uniquely
  distinguish those two because publisher-side monotonic counters were absent.
- The base watchdog's trigger was stale NDT. The deeper reason NDT became stale
  while raw LiDAR was fresh remains unconfirmed; do not change its 300 ms
  freshness rule or synthesize heartbeat traffic.
- The pre-failure implementation placed all base-watchdog subscriptions, the
  freshness timer, and status publication on one `spin_once`/default callback
  path; status publication also held the shared state lock. This is a concrete
  scheduling vulnerability consistent with the ~0.12–0.23 s input callback
  gaps immediately preceding the run, but the old evidence did not include
  base timer/publish-call timing during the 0.31 s quiet interval, so it cannot
  prove this was the sole runtime trigger.

Targeted correction and instrumentation now in the development tree:

- Base watchdog timer and input callbacks have distinct callback groups; the
  localization watchdog gives NDT its own group. Multi-threaded executors keep
  safety timers independently schedulable. Existing freshness values and
  heartbeat-withholding behavior are unchanged.
- The full-stack `atlas_perception_planning_demo/perception_guard.py` now separates base heartbeat/status callbacks,
  sensor/odometry callbacks, and its safety timer into three callback groups
  serviced by a three-thread executor. A lock protects shared safety state.
  Its diagnostic status includes bounded base callback counts, monotonic
  callback gaps/durations, and ROS source age at callback. The base watchdog
  status also includes bounded timer and heartbeat/status publication timing.
  Both watchdog status publishers now release the shared state lock before the
  RMW status publication call; the safety heartbeat decision remains protected
  and the perception heartbeat is emitted atomically with its freshness check.
  The parallel `atlas_obstacle_stopping_demo` guard received the same isolation
  and diagnostics for consistency, but the five-lap launch uses the perception
  package guard.
- Added regressions cover callback-group separation, the fail-safe NDT latch
  withholding heartbeat while still publishing status, callback timing, and
  diagnostic propagation. No watchdog threshold or authorization rule changed.

Verification:

- Focused suite passed 51/51: watchdog health 12, localization watchdog 7,
  full-stack perception guard 5, safety source labels 1, scene 15, loop map 2,
  lap metrics 9.
- `py_compile` passed when its cache was directed to `/tmp` (the source
  worktree does not permit Python cache writes). `git diff --check` passed.
- One-job direct CMake configure/build/install passed for
  `atlas_closed_loop_demo`, `atlas_localization_closed_loop_demo`,
  `atlas_obstacle_stopping_demo`, `atlas_perception_planning_demo`, and
  `atlas_full_stack_demo` into
  `/tmp/atlas_basehb_install_20261031_v2`. Installed watchdog, guard, and
  validator files match source. `ros2 pkg prefix` resolves all five packages
  to this prefix. Five-lap launch `--show-args` resolves and parses successfully
  with `ROS_LOG_DIR` redirected under `/tmp`.
- No live ROS re-run was attempted here. ROS launch logging under the default
  `~/.ros/log` is read-only in this sandbox, and the previously tested socket
  networking restriction remains. Host runtime confirmation is required.

Next host action (choose a ROS domain known unused on that host):

```bash
source /home/marian/roam/atlas_autoware_dev_env.sh
source /tmp/atlas_five_lap_diag_20261031_d/install/setup.bash
source /tmp/atlas_five_lap_diag_20261031_e/install/setup.bash
export AMENT_PREFIX_PATH=/tmp/atlas_basehb_install_20261031_v2:$AMENT_PREFIX_PATH
export ATLAS_AUTOWARE_ROOT=/home/marian/roam/references/autoware
export ROS_DOMAIN_ID=226
export ROS_LOG_DIR="/tmp/atlas_five_lap_host_logs_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$ROS_LOG_DIR"
python3 /home/marian/roam/atlas_autoware_dev/src/atlas_full_stack_demo/scripts/run_lap_validation.py \
  --laps 1 --timeout 240 \
  --output "/tmp/atlas_five_lap_host_1_$(date -u +%Y%m%dT%H%M%SZ)"
```

This install prefix is direct-CMake and is intentionally selected by
`AMENT_PREFIX_PATH` (package-prefix checks resolved all five packages to it).
Inspect `safety_status.jsonl`: the localization watchdog's
`publisher_timing` and the full-stack guard's `base_callback_diagnostics` now
show publisher attempts/call duration and subscriber callback counts/gaps.
Keep the result even if it fails. Do not attempt two/five laps until the
one-lap physical route, terminal stop/hold, and no-latch criteria pass.

## New host run: base path fresh; guard missed predicted objects

Evidence: `/tmp/atlas_five_lap_host_1_20261031T165002Z/`. This run traversed
55.052 m of truth distance (0.8681 EKF-projected lap), with 0 completed laps.
It failed at the perception guard on `predicted_objects`; terminal hold remained
0 s. Localization 3D RMSE was 0.07145 m, cross-track RMS 0.03354 m, and heading
RMS 0.03064 rad.

This run did **not** reproduce a base heartbeat/status stale fault. At the guard
fault, base heartbeat and status source ages were 0.0149/0.0148 s and receipt
ages were 0.00079/0.00022 s. The base watchdog had published 3,068 heartbeats;
the guard's observed maximum base callback gap was 0.186 s, below the unchanged
0.300 s freshness limit. The previous base fault path is operationally improved
in this run, though a single run does not establish reliability.

The new failure is localized more strongly: the guard fault snapshot still had
predicted-object source stamp `1793465469.5691535`, age 0.3025 s. The independent
validator had already received the same predicted-object stream's next sample
at stamp `1793465469.6697211` at monotonic 330938.7743; the guard emitted its
fault status at monotonic 330938.8351. Thus this was not simply a stopped
predicted-object publisher. The message was visible to another subscriber
before the safety tick, while the guard's cached sample remained older. The
guard previously put several large PointCloud2 subscriptions and predicted
objects in one mutually exclusive callback group, making subscriber servicing
contention a supported cause. Raw and detected-object counts were both 653;
predicted-object count was 647. The run also logged an EKF twist queue overflow
and 0.132 s pose-smoothing warning, consistent with high scheduling load.

Targeted follow-up:

- Predicted-object subscription is now in a dedicated mutually exclusive
  callback group, separate from large cloud deserialization callbacks, the
  base authorization stream, and the 20 ms timer. The guard uses four executor
  workers for these four groups. It records predicted-object callback count,
  maximum monotonic receipt gap, and callback duration in its status output.
- The localization watchdog overrides the base `publish_status`; it had not
  advanced the inherited status-publisher counter/timing fields, leaving the
  reported `status_publish_count` at 1. It now updates those counters and call
  duration after publication, so the next run can distinguish a stopped status
  publisher from a subscriber-side gap. Status RMW call duration is now measured
  separately from status-JSON preparation/lock time. This is diagnostics-only.

Verification after these changes: 52/52 focused tests passed (watchdog health
12, localization watchdog 7, perception guard 6, source labels 1, scene 15,
map 2, lap metrics 9). One-job CMake install passed for the localization and
perception packages into `/tmp/atlas_basehb_install_20261031_v2`; installed
scripts match source and package prefix resolution points there. Python syntax
and `git diff --check` passed. No new ROS campaign was attempted here.

The inherited localization watchdog's status counter and publisher-call timing
were subsequently corrected and reinstalled into the same prefix. Its exact
RMW call duration is now distinct from status preparation/lock time; the full
52-test suite, Python compilation, installed-source comparison, and
`git diff --check` passed again. No safety behavior changed.

Next host action remains the one-lap run above, using the rebuilt v2 prefix.
Inspect `predicted_objects_callback` in perception-guard statuses alongside the
validator's predicted-object receipt records. If the guard count/gap aligns
with the validator, but the source stamp itself pauses, investigate the
perception output component. If the validator receives fresh stamps while the
guard callback counter/gap pauses, inspect DDS reader/executor servicing next.
The single-threaded validation monitor and EKF queue warnings are potential
resource contributors but are not accepted as root cause without a correlated
reproduction.

## Experiment 2026-11-07 — Five-lap downstream freshness gap

Evidence: `/tmp/atlas_five_lap_test_20261107T043622Z_diag_427812`.
At perception latch ROS time `1794026258.7970955`, the last predicted-object
sample still had source stamp `1794026258.4819193`; it was only 39 ms old when
received, then had no later guard callback for 276 ms. At the same latch, raw,
extended, deskewed, compact, and obstacle point-cloud inputs had advanced to
`1794026258.682819` (source age 114 ms). The localization watchdog then
latched NDT at `1794026258.8515367`, also last stamped `1794026258.4819193`.
This confirms a downstream output gap while raw/intermediate clouds had
recovered; it does not identify whether detector/NDT processing, delivery, or
executor scheduling caused the gap.

The validation observer itself was behind: scene-timing diagnostics near the
end were collected about 2 seconds after their callback monotonic stamps, and
its receipt/header-age maxima therefore are not transport-arrival measurements.
Scene instrumentation in its captured window showed max timer scheduling
17.4 ms, max callback duration 63.0 ms, max scan-generation 16.4 ms, and max
scan-publish gap 123.2 ms. The five-lap route had 1,809 points; its repeated
trajectory publication averaged 13.9 ms and peaked at 61.3 ms. Available host
memory sampled by the scene fell to 74 MB; 1-minute load peaked at 11.15.
Successful one-lap reference had 381 route points and minimum sampled available
memory 158 MB (maximum 264 MB). Historical swap usage and process-level system CPU were not
recorded. RViz was disabled, but the visualization-only ego marker remained
enabled by launch default.

Diagnostic-only follow-up added a detected-object callback counter/gap/age to
perception-guard status and an NDT execution-time callback counter/gap/value to
localization-watchdog status. Neither stream participates in stale-input
checks. The lap-validation launch now disables the visualization-only ego
marker by default when it already disables RViz; `--launch-ego-marker` opts in.

Verification: focused tests passed (7 perception guard, 9 localization
watchdog, 2 validation-launch/timing tests); Python compilation and
`git diff --check` passed. Isolated one-worker build/install passed for
`atlas_closed_loop_demo`, `atlas_localization_closed_loop_demo`,
`atlas_perception_planning_demo`, `atlas_map_integration_demo`, and
`atlas_full_stack_demo` under `/tmp/atlas_fivelap_diag_followup_20261107_01`.
Installed diagnostic/runner scripts match source hashes. No ROS runtime test
was run in the restricted session. Next host run: use this isolated overlay,
`run_lap_validation.py --laps 5 --timeout 700`, with a unique output directory;
compare detected-object, predicted-object, NDT-execution, NDT-pose, and cloud
stage counters at the first stale event. Keep all 300 ms freshness limits
unchanged. Do not claim root cause until those new stage observations separate
processing gaps from per-node delivery/scheduling gaps.
