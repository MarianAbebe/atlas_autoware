# Obstacle-stop watchdog investigation — unresolved

2026-10-22, ARM64 / ROS 2 Humble / 4 GB VM.
**The 11/11 completion criterion is not met.** The earlier complete, serial suite
passed **9/11**: off-route handling and the odometry-drop recording's command
provenance check failed. No justified safety/algorithm fix was identified or
applied. Localization integration was not started.

The subsequent [timeboxed follow-up](timeboxed_followup/REPORT.md) passed
**10/11**, with nominal driving still failing. It adds middleware timing evidence
and a deterministic raw-observer backlog experiment; no runtime fix was applied.

A later [simulation timing architecture audit](../simulation_timing/REPORT.md)
proves scheduling-sensitive simulator steps and recorder loss mechanisms in
isolated tests. It does not resolve the historical suite failures or change
watchdog safety behavior.

## What is established

The unexpected latches are actual freshness violations under the existing rule:
each required input must have a source stamp age in `[0,0.3)` seconds and a
monotonic receipt age below 0.3 s. The latch persists after data recovers and
requires explicit rearm, as intended. No evidence identifies missing messages
at startup, future timestamps, malformed content or a changed threshold as the
cause of these particular latches. Producer delay versus queued delivery remains
unresolved; those are different causes with different remedies.

Diagnostic-only fields now freeze the source stamp, source age, receipt age and
watchdog tick gap at the first decision. Source ages and receipt ages are measured
from the same clock reads used by the unchanged stale decision. Two representative
new failures:

| Run | Decision ROS timestamp | Inputs over source-age bound | Source age | Receipt age | Watchdog tick gap |
|---|---|---|---|---|---|
| independent nominal | 1792635505.2589955 | steering / acceleration | 0.304699 / 0.304769 s | 0.097708 / 0.105319 s | 0.029635 s |
| serial suite off-route | 1792636572.9885602 | odometry / steering / acceleration | 0.304841 / 0.304650 / 0.304720 s | 0.132845 / 0.132757 / 0.132618 s | 0.016669 s |

The monitor was executing regularly in these examples; fresh *receipts* did not
make old source stamps acceptable. This establishes the immediate latch condition,
not the mechanism delaying the source data. The downstream obstacle guard then
withheld authorization because the original watchdog was no longer armed.

The earlier real-perception milestone's three failures remain archived in
[its original summary](../../real_data/engg_loading_dock_01/perception_regressions/summary.json):

| Scenario | First fault ROS timestamp | First recorded latch receipt | Required feedback named by base watchdog |
|---|---|---|---|
| none | 1792633815.0291722 | 40.5028 s | odometry, steering, acceleration, trajectory, command |
| ahead | 1792633964.7916443 | 20.6224 s | steering, acceleration |
| off_route | 1792633994.4874868 | 11.6364 s | odometry, steering, acceleration, command |

Those old recordings did not contain per-input ages at the watchdog's decision;
exact ages cannot be reconstructed reliably from another subscriber's receipts.
The new snapshots close that evidence gap without changing the decision.

## Comparisons and hypotheses tested

The 2026-10-20 passing recordings in
`src/atlas_obstacle_stopping_demo/results/final` had maximum odometry arrival gaps
of approximately 56–65 ms and delivered-command gaps of approximately 52–93 ms.
The earlier failing nominal run reached 755 ms and 1.072 s respectively. Failure
locations and affected topics vary across repetitions, rather than consistently
occurring at a particular route feature.

The original closed-loop config/launch/scripts matched the previous passing
source hashes before diagnostic instrumentation. The only obstacle-demo source
hash difference was `run_suite.py`'s previously added results-directory handling;
independent `run_test.sh` calls also fail, so suite enumeration alone cannot
explain the problem. The real-data harness uses a separate executable, namespace
and domain; it is not loaded by `run_test.sh`. Simulator reports explicitly record
synthetic perception and `real_lidar_executed=false`. No shared input/state link
from recorded detections to simulator pose was found.

The runtime tracer records CPU counters, per-process CPU/RSS/thread counts,
pressure/memory counters, monotonic sampling gaps, realtime-versus-monotonic
offsets and relevant environment variables. Raw traces remain at the paths in
[summary.json](summary.json); short fault-window excerpts are archived alongside
each experiment's report.

Findings do **not** establish CPU contention as the unique root cause:

- A failing nominal trace reached about 98% global CPU busy in sampled intervals.
  Passing ahead/off-route traces also reached about 92–93%.
- There was at least approximately 1 GiB MemAvailable in the initial failing
  nominal trace. The affinity trial dipped lower, to about 622 MiB. This is not
  evidence of an OOM-induced crash; reports found no crash. Memory pressure and
  CPU utilization alone do not identify the delayed hop.
- Realtime-minus-monotonic offset variation stayed below 0.91 ms in the sampled
  diagnostic traces, and below 0.42 ms near inspected faults. A clock step of
  hundreds of milliseconds is not supported by these measurements.
- BLAS/OpenMP thread limits did not prevent a nominal latch at 76.9 s. They were
  not retained as a solution.
- Separating recording onto CPU 5 and the ROS graph onto CPUs 0–4 did not prevent
  an off-route command-age latch at 85.6 s. The experimental affinity option was
  removed from both validator and suite driver.
- A separate bounded load experiment ran eight CPU workers for eight seconds
  starting at 20 s; measured CPU busy reached 100%, but no watchdog latched in
  its 45-second measurement. Its route/hold checks failed because that diagnostic
  measurement was intentionally short. It is **not** a standard suite pass and
  does not rule out a timing-sensitive resource problem in another interval.
- A passive depth-one observer was attached to original simulator outputs,
  post-relay feedback, controller output and final trajectory. That nominal run
  passed, so it did not capture a failing hop. Its CSV remains at
  `/tmp/atlas_watchdog_hops.csv`. Do not infer a queue defect from a healthy trace.
- Growing visualization-path serialization is measurable (about 13 ms for 4,500
  poses in a separate serialization microbenchmark), but causal attribution to
  the latches was not established. No visualization behavior was changed.

The evidence is consistent with intermittent end-to-end feedback delay. It does
not distinguish producer scheduling, relay backlog, middleware behavior or
consumer scheduling sufficiently to justify a production change. No speculative
QoS/history change, restamping, grace period or threshold increase was applied.

## Executed validation

[summary.json](summary.json) and per-experiment folders contain the reports,
first latch snapshots, source hashes and original thresholds.

| Independent / diagnostic execution | Result |
|---|---|
| nominal, original scheduling | FAIL: steering/acceleration source age |
| ahead obstacle, original scheduling | PASS |
| off-route, original scheduling | PASS |
| nominal with BLAS/OpenMP thread limits | FAIL: command freshness |
| off-route with observer/runtime CPU separation | FAIL: command freshness |
| nominal with passive latest-sample observer | PASS |
| bounded CPU-load diagnostic, 45 s measurement | No latch; incomplete-route checks failed as expected |

Latest complete serial suite (all eleven cases executed):

| Case | Result |
|---|---|
| none | PASS |
| ahead | PASS |
| off_route | FAIL: route completion, nominal authorization, terminal hold |
| stale_drop / stale_freeze | PASS / PASS |
| feedback_drop / feedback_freeze — odometry | FAIL / PASS |
| feedback_drop / feedback_freeze — steering | PASS / PASS |
| feedback_drop / feedback_freeze — command | PASS / PASS |

All eight deliberately stale/missing/frozen cases passed their expected latch,
braking, retained-stop and invalid-rearm checks. The odometry-drop case failed
**only** `delivered_commands_match_pinned_controller`: one delivered payload
was absent from the validator's raw-controller recording. There were 879 raw
rows and 878 delivered rows; the unmatched source stamp was 1792636724.064528.
Recorded raw neighbors were 1792636724.034619 and 1792636724.0943687. See
[command_recording_mismatch.json](command_recording_mismatch.json).
An observer recording gap is possible, but not proved; the assertion remains
unchanged and the case remains failed. The suite returned nonzero, and a passed
whole-suite offline review is not claimed.

Preservation checks after the investigation:

- Synthetic perception CTest: **PASS**, including three synthetic scans through
  the existing converter/deskew/clustering chain. [Log](synthetic_perception_ctest.txt).
- Four pure watchdog decision tests: **PASS**, covering source/receipt boundary
  checks, missing/future/frozen stamps, base authorization, and frozen diagnostic
  snapshots. [Log](watchdog_health_tests.txt).
- Real perception confirmation: **five recorded scans PASS**, with the same
  filtered counts 74/47/77/103/98 and bypass counts 67/44/70/93/86. All point,
  membership, size and header checks remain active.
  [Evidence](real_perception_confirmation/results.json).

## Retained changes and reproduction

Only diagnostic instrumentation and investigative tools/tests are retained:

- `feedback_watchdog.py`: immutable first-latch input ages and tick-gap snapshot.
- `obstacle_guard.py`: immutable first-latch input-age snapshot.
- `scripts/trace_regression_runtime.py`: default passive process/CPU/clock tracer;
  optional explicitly selected load control is bounded to eight workers/ten s.
- `scripts/watchdog_topic_probe.py`: bounded, publish-free latest-sample observer.
- `test_watchdog_health.py`: actual decision-method boundary tests.

[policy_checks.json](policy_checks.json) records identical AST return expressions
for the old and new stale methods. All threshold files are preserved. No pinned
Autoware, rosbag, converter or clustering implementation was changed. No packages
were installed. Installed Python copies were refreshed to run the diagnostics.

From the Atlas root, with the existing caches:

```bash
# Independent reproduction and resource trace; choose a fresh trace directory.
/usr/bin/python3 scripts/trace_regression_runtime.py \
  --output-dir /tmp/atlas_watchdog_trace_new -- \
  bash src/atlas_obstacle_stopping_demo/scripts/run_test.sh \
    --scenario off_route --results-root /tmp/atlas_watchdog_runs_new
# Complete original suite; run serially, after builds/perception work ends.
/usr/bin/python3 scripts/trace_regression_runtime.py \
  --output-dir /tmp/atlas_watchdog_suite_trace_new -- \
  python3 src/atlas_obstacle_stopping_demo/scripts/run_suite.py \
    --results-root /tmp/atlas_watchdog_suite_new
# Pure decision tests (no ROS graph).
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 \
  src/atlas_obstacle_stopping_demo/scripts/test_watchdog_health.py
```

The passive probe must run in the same sandbox/process environment and ROS
domain as the simulator. Start it alongside `run_test.sh` with ROS_DOMAIN_ID=80,
using `--output /tmp/fresh.csv --duration 140`; it creates subscriptions only and
writes original source stamps plus ROS/monotonic receipt times. A probe run
without a failure cannot settle the delayed-hop question.

## Remaining blocker

Capture an **unrequested latch with the per-hop observer attached**, and compare
its last pre-decision source stamps with the frozen watchdog snapshot. That
would distinguish unavailable producer data from data waiting in a relay/reader
queue. Correlate that event with host-level scheduling/middleware diagnostics
before selecting a change. The old traces cannot identify the original external
CPU actor or prove the specific delayed hop.

The exact root cause remains unestablished. The safety boundary is functioning
as configured; weakening it would conceal the failure. The project should not
claim 11/11 or proceed to localization integration on the strength of these runs.
