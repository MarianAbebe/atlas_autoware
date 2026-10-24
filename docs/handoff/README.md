# ROAM Atlas Saturday handoff

Status date: 2026-10-23 UTC

This directory is the operator and developer handoff for the experimental Atlas
full-stack simulation. For a new Ubuntu 22.04 x86_64 laptop, start with
[NATIVE_UBUNTU_TESTING.md](NATIVE_UBUNTU_TESTING.md). The existing
[QUICK_START.md](QUICK_START.md) documents the reference ARM64 VM.

## Readiness statement

The current Ubuntu 22.04 ARM64 VM can reproducibly launch the stack, initialize
localization without engagement, and run the complete stationary data path. A
190.57 second synchronized recording showed NDT, EKF, perception, planning and
control publication with both watchdogs armed and unlatched for more than 103
seconds.

This is **not** verification of engaged autonomous operation, physical vehicle
operation, real-campus-PCD localization, or hardware readiness. Earlier engaged
runs were intermittent, and a prior stationary run latched both safety monitors
during a graph-wide scheduling interruption. The most recent stationary run
also contained a smaller event that brought the bag-observable NDT age to about
253 ms against the unchanged 300 ms safety limit.

## Documents

- [Quick start](QUICK_START.md): build, launch, safe localization initialization,
  inspection and shutdown.
- [Architecture](ARCHITECTURE.md): ownership, components, frames and data flow.
- [Validation](VALIDATION.md): current evidence, historical failures and limits.
- [Troubleshooting](TROUBLESHOOTING.md): safe diagnosis without bypassing latches.
- [Continuation plan](CONTINUATION_PLAN.md): prioritized work and acceptance criteria.
- [Repository audit](REPOSITORY_AUDIT.md): portability and reproducibility blockers.
- [Native Ubuntu testing](NATIVE_UBUNTU_TESTING.md): pinned source, external map
  assets, build, no-graph smoke test and disengaged stationary validation.
- [Provisional requirements](PDR_ALIGNED_REQUIREMENTS.md): evidence-backed
  capability matrix pending the official ROAM PDR.

The package-specific implementation guide remains at
[`src/atlas_full_stack_demo/README.md`](../../src/atlas_full_stack_demo/README.md).

## Non-negotiable operating restrictions

- Run only one `/atlas_loc` graph in ROS domain 86.
- Never weaken the 300 ms freshness checks or bypass either safety monitor.
- Never use simulator-truth odometry as controller or localization input.
- Do not treat a watchdog-induced stop as a planned obstacle stop.
- Preserve every failed run; do not rearm merely to continue a demonstration.
- The stationary quick start does not execute `manual_start.py`, because that
  helper publishes an engagement request.
- Do not operate physical hardware from this experimental stack.
