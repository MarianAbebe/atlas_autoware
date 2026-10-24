# Localization-driven simulation — milestone incomplete

Executed 2026-10-22, Ubuntu 22.04 ARM64 / ROS 2 Humble, 4 GB VM.

**The actual pinned NDT → EKF → MPC/PID → gate → simulator chain executed. Several
8 m synthetic drives completed with a ten-second stopped hold. Repeatable nominal
completion is NOT established. The retained immediate-scan configuration has
failed its nominal route test because freshness latches stop the vehicle.**
No safety threshold or convergence gate was relaxed. No truth fallback,
replacement estimator, upstream changes, system installations, or research-bag
changes were made. The original milestone did not invoke or change the
obstacle-stop suite; subsequent regression reruns are recorded separately below.

A subsequent [timing architecture investigation](../../docs/regressions/simulation_timing/REPORT.md)
adds an isolated controlled-step proof and identifies clock-domain blockers to a
blanket `/clock` switch. The integration results below remain unchanged.

## Architecture and evidence of state origin

See the [data-flow diagram and message contracts](README.md). This package reuses
the existing simulator, MPC/PID controller, gate, model values and Analytical
velocity smoother. Its only new online functions are virtual sensor generation,
fixed short-route input, activation/engagement scaffolding, independent evaluation,
and an extension of the existing simulation freshness monitor.

[Captured graph](results/final_no_scan_delay/nominal/graph.json) verifies:

- `/atlas_loc/ekf/ekf_odom` has exactly one publisher:
  `/atlas_loc/ekf/ekf_localizer`; the controller and gate subscribe to that topic.
- NDT prediction poses originate only from the pinned EKF.
- EKF pose measurements originate only from the pinned NDT.
- Scan messages originate only from the sensor fixture and reach NDT.
- Controller, gate, smoother, NDT and EKF do not subscribe to simulator truth
  odometry. Truth has a distinct body frame, `hidden_truth_base`; estimated
  `map → base_link` is published by EKF, with no truth transform on that edge.
- The pinned gate is the exclusive command publisher to the simulator.

The input scan acquisition stamps match NDT output stamps. Three sampled points
per scan, independently transformed with recorded simulator truth, remain fixed
in the world within approximately 1 micrometre in the completed-drive capture.
Sensor coordinates change by 7.924 m during that drive. This establishes evolving
pose-dependent scan generation rather than replay of a fixed scan or injection
of per-scan truth pose seeds. EKF receives only one biased initial pose at
(100.3,100.2,0), then NDT poses plus ideal body speed/yaw-rate measurements.

## Bounded successful integration level

[development_03](results/development_03/validation.json) completed the 8 m route
and full ten-second hold with no freshness latch. It used the initial sensor
fixture's explicit 40 ms scan-publication wait. The retained fixture removes
that unnecessary wait; this earlier success is **not** presented as a passing
nominal result for the retained configuration.

| Measurement in that completed drive | Result |
|---|---:|
| Travel / final goal distance | 7.923607 m / 0.076443 m |
| Maximum lateral error | 0.002984 m |
| Maximum speed / acceleration magnitude | 0.782839 m/s / 0.301329 m/s² |
| NDT post-engagement max / RMS XY error | 0.007348 / 0.003111 m |
| EKF post-engagement max / RMS XY error, interpolated truth | 0.008634 / 0.005146 m |
| NDT / EKF max 3D position error | 0.008921 / 0.009039 m |
| NDT / EKF max yaw error | 0.000104 / 0.000024 rad |
| NDT / EKF / controller observed rates | 8.94 / 50.00 / 33.33 Hz |
| Accepted NDT outputs / recorded scans | 238 / 244 |
| Minimum NDT likelihood / maximum iterations | 3.475 / 6 |
| Maximum reported NDT execution time | 13.322 ms |
| Sampled process-group aggregate RSS | 381,944 KiB (about 373 MiB) |

[Offline evaluation](results/development_03/analysis.json) interpolates independent
50 Hz truth between adjacent samples, excluding brackets over 40 ms. The largest
used bracket in the completed drive was about 22 ms. The runtime validator uses
nearest truth within 25 ms; both methods and counts are retained. Aggregate RSS
sums processes and double-counts shared pages; it is sampled, excludes the
external evaluator, and is not an exact peak physical-memory measurement.

The numerical errors describe a noise-free synthetic map/model, not measured
Atlas accuracy, campus PCD registration, or physical localization performance.
Startup convergence errors are excluded from driving-error metrics. Initial
biased estimates are never used as repeated map poses or truth corrections.

## Development failures and nominal timing blocker

The 3 m fixture first overshot by 0.293 m with a handwritten velocity profile;
using the existing pinned smoother instead stopped about 0.217 m short. Both
failures remain in `development_01` and `development_02`. The test then used an
8 m straight route and the existing 1 m/s cap, with unchanged controller stopping
parameters, goal tolerance (0.15 m), speed/acceleration limits and ten-second hold.

The first complete scenario suite [returned 4/5](results/final/suite.json).
Its nominal run failed after an unrequested NDT-freshness latch:

- Decision ROS timestamp **1792640319.3673868**; NDT source age **310.230 ms**,
  receipt age **124.969 ms**; watchdog tick gap **19.111 ms**.
- EKF odometry source age was **7.572 ms**. EKF continued publishing current
  predictions; monitoring EKF timestamps alone would miss absolute-pose staleness.
- The independent evaluator had recorded a newer NDT result shortly before the
  latch, but this does not prove when the watchdog's own reader received it.
  Reader delivery, executor ordering and external scheduling remain unseparated.

Inspection of pinned `SmartPoseBuffer::interpolate` established that a scan newer
than the latest EKF prediction is allowed within its unchanged timestamp tolerance.
The fixture's artificial 40 ms wait was removed, retaining acquisition timestamps.
This was a contract/latency correction, **not a proven cure for the VM interruption**.

The immediate-scan suite [returned 3/5 before validator review](results/final_no_scan_delay/suite.json):
nominal timing failed, and an EKF-shutdown first-fault assertion was incorrect
(see below). The nominal snapshot recorded trajectory/NDT ages **310.363 / 318.758
ms**, with a **244.979 ms watchdog tick gap**. The gate stopped the vehicle after
2.465 m, well before the 8 m goal. NDT accepted poses remained numerically accurate;
that does not make stale controller-visible localization acceptable. This repeat
proves that removing the artificial wait did not establish nominal reliability.

The exact initiating cause of these interruptions remains unproved. Do not claim
CPU contention, middleware loss, clock discontinuity or a particular reader race
as the established cause. No runtime fix to the safety behavior was attempted.

## Localization loss and unavailable-output handling

The first complete suite's four deliberate failure cases all passed:

| Case | Latch latency | Stop distance | Settling | Result |
|---|---:|---:|---:|---|
| Stop NDT scans during driving | 0.249 s | 0.625 m | 1.701 s | PASS |
| Deactivate EKF during driving | 0.312 s | 0.728 m | 1.833 s | PASS |
| NDT unavailable at startup | no authorization | 0 m travel | remains stationary | PASS |
| EKF unavailable at startup | no authorization | 0 m travel | remains stationary | PASS |

The immediate-scan repeat also stopped safely on both runtime faults (NDT drop:
0.331 s, 0.606 m; EKF shutdown: 0.293 s first latch, 0.723 m). Both unavailable
startup cases again travelled zero metres. Scan recovery did not automatically
rearm safety; EKF predictions after scan loss were not treated as renewed NDT
measurements. No post-measurement cleanup sample counts as a stopping result.

### Corrected validator ordering assumption

In the immediate-scan EKF-shutdown run, NDT became stale first: its immutable
first-latch input list contained `ndt`, while EKF odometry was only 287 ms old.
The original evaluator incorrectly demanded `odometry` in that first list.
Captured `currently_stale` status included odometry **297 ms** after injection.
All braking/stop/no-restart checks already passed.

The corrected evaluator independently requires the injected stream to become
stale within the unchanged **0.4 s evaluation bound**, a timely latch, safe stop,
and no pre-injection unrequested latch. It no longer assumes which dependent
stream expires first. This changes test observation logic only. Four deterministic
tests cover NDT-first/odometry-next ordering, wrong input, late/pre-injection
observations, and missing injection. The [offline ordering review](results/final_no_scan_delay/drop_ekf/ordering_review.json)
preserves the original failed verdict and separately records the corrected check;
it is not silently rewritten as a new runtime pass.

## Existing regressions and final targeted checks

The final targeted EKF-shutdown [retest passed](results/ordering_fix_retest/validation.json)
with the corrected validator and retained immediate-scan runtime: **0.321 s**
latch latency, **0.710 m** stop distance, **1.839 s** settling, zero final speed,
and no restart. Timestamp/frame, scan-origin, EKF-origin, convergence, numerical,
control-bound and pre-injection-health checks all passed. The 8 m nominal drive
was not repeated after this evaluator-only correction; the nominal timing blocker
remains unresolved.

| Existing regression / decision coverage | Result |
|---|---|
| Standalone NDT, direct and full preprocessing chain | **2/2 PASS** |
| NDT-aware watchdog decision tests | **4/4 PASS** |
| First-fault observation-order tests | **4/4 PASS** |
| Original campus nominal drive | **FAIL** |
| Original odometry / steering / command drops | **3/3 PASS** |
| Original odometry / steering / command freezes | **3/3 FAIL** |

The original control regressions therefore passed **3/7**, not a clean
preservation result. [Raw regression summary](results/regressions/summary.json),
[fault suite](results/regressions/closed_loop_faults/suite.json), and per-run
verdicts preserve all failures. No obstacle regression was substituted.

The original campus nominal run latched on **all five required feedback inputs**
with no NDT/EKF/localization-fixture processes running. Its failed checks also
include route/terminal completion, controller frequency and command provenance.
The freeze cases failed these checks:

- Odometry freeze: actual acceleration bound and command provenance.
- Steering freeze: controller frequency and command provenance.
- Command freeze: controller frequency and controller response to live speed.

These are reported as unresolved original-regression failures. Reused runtime
sources and safety thresholds were not modified to make them pass. This evidence
does not prove that all failures have a common external cause or establish the
specific scheduling/transport fault. The new localization runtime remains isolated
in domain 83; original control tests run serially in domain 79 and standalone NDT
in domain 84.

Eight deterministic decision/observation tests and the standalone NDT tests pass,
but **neither the retained localization nominal test nor the original control
regression suite passes in this session**. The milestone is incomplete.

## Reproduction and limits

Use [README.md](README.md) and the scripts beside it. Missing pinned localization
packages were built in `.localization_cache` with one compiler job. Existing
controller/simulator/perception dependency caches were reused. Original pinned
repositories remained clean; [revision/status evidence](results/pinned_sources.json)
records root 1.9.0, Core, Universe and simulator revisions. The research bag was
never used by this milestone.

Remaining blockers, in priority order:

1. Establish a stable end-to-end localization/control timing environment and
   capture actual reader receipt/callback timing at an unrequested latch. The
   retained nominal loop has not passed route completion. Do not relax the
   300 ms freshness rule to conceal this failure.
2. Resolve current failures of the unchanged campus control regressions; existing
   code and thresholds were preserved, but a clean fresh regression result is
   not available. The obstacle-stop investigation remains separate and unresolved.
3. Only after short nominal operation is repeatable, extend coverage to curved
   routes, sensor noise/bias, larger initialization errors and longer runs.
4. Campus-map driving requires defensible scan coverage, virtual geometry and
   PCD/Lanelet2 registration. Regional overlap is not verified alignment; no full
   campus NDT loop or research-recording localization is claimed.

The LiDAR model is an instantaneous inverse transform of a downsampled static
surface, with no occlusion or rolling acquisition. No deskew claim is made.
Ideal wheel-speed/gyro and actuator sensors, assumed covariances, provisional
geometry, supplied startup pose, and ideal geared dynamics limit generality.
The measured stop cases are bounded synthetic experiments, not physical-vehicle
or worst-case safety certification.


Subsequent simulation-timing follow-up: the original campus regressions freshly
passed 7/7 and the separate localization suite passed 4/5; nominal NDT/EKF
driving remains blocked by a measured freshness latch before engagement. See
[the follow-up report](../../docs/regressions/simulation_timing/IMPROVEMENTS.md)
for evidence, the command observation correction and isolated fixed-step scope.


## Nominal repeatability follow-up, 2026-10-22

[Bounded investigation and all attempts](../../docs/regressions/localization_nominal_repro/REPORT.md):
two identical baseline attempts produced **1/2 passes**. After correcting a
confirmed evaluator RSS-sampling cadence bug, three identical-source attempts
produced **2/3 passes**, each passing attempt including the full ten-second hold.
This does **not** establish nominal reproducibility or a causal timing cure.

The two fresh passing final errors were 0.105552 m and 0.076627 m. Post-engagement
NDT max XY errors were 0.007812 m and 0.008047 m; EKF maxima were 0.009193 m and
0.009547 m, against independently recorded ideal simulator truth. All attempts
retain exclusive EKF odometry at the controller, exclusive NDT pose measurements
at EKF, and no truth-odometry fallback.

The failed baseline records a 332 ms watchdog gap while independent producers
continue. The failed corrected attempt records a timely 20 ms watchdog tick but
a 340 ms observed scan/NDT gap. The latter latches on NDT source age 314 ms and
stops short of the goal. Exact watchdog-reader delivery versus callback dispatch,
fixture execution and external scheduling remain unseparated. Whole-graph
fixed-step execution still requires changes to clock contracts beyond this scope;
no blanket `use_sim_time` override was attempted.

The small correction affects resource observation only; simulator, fixture,
NDT, EKF, controller, gate and watchdog implementations/settings remain unchanged.
The three new deterministic cadence tests pass. NDT dropout and both unavailable
startup cases pass; the initial EKF-drop attempt fails to engage and therefore
does not exercise its requested fault. The linked report contains the final
preservation-suite and any isolated follow-up results without hiding failures.


Final preservation results for that follow-up: **original control 7/7**, standalone
NDT **2/2**, obstacle-stop **10/11**. Off-route obstacle driving failed on a
328 ms trajectory-freshness decision, so the previous 11/11 result did not repeat.
All eight deliberate obstacle/feedback faults passed. A single isolated EKF-drop
follow-up successfully engaged, injected shutdown and passed its stop/latch checks;
the earlier pre-injection failure remains recorded. Nominal repeatability and
reliable startup are still unproven. Work stopped within the 30-minute timebox.


The subsequent [experimental full-stack composition](../../docs/full_stack/REPORT.md)
connects preprocessing, NDT/EKF, perception and pinned obstacle stopping in a
synthetic world located in campus-vector coordinates. Its results are separate:
obstacle 1/2, clear 2/2, isolated NDT loss 2/2, shared-LiDAR loss 2/2. The default
localization-only launch smoke passed after the composition API change. Existing
scheduling/reader gaps and original-PCD registration remain unresolved.
