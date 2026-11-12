# Native Ubuntu Ackermann Nav2 validation

This guide reproduces the isolated Ackermann Nav2 simulation on Ubuntu 22.04
with ROS 2 Humble. It does not claim that native Ubuntu removes the timing
interruption observed in the ARM64 VM. It does not authorize physical vehicle
operation.

## Get the development branch

Use a separate development checkout. Do not use or modify the stable checkout.

```bash
git clone --branch feature/prd-gap-closure --single-branch \
  https://github.com/MarianAbebe/atlas_autoware.git atlas_autoware_dev
cd atlas_autoware_dev
git status --short --branch
```

After the checkpoint is pushed, confirm the branch is at the checkpoint commit
and the worktree is clean before continuing.

## Install dependencies

Install ROS 2 Humble from its official Ubuntu Jammy repository. The simulation
uses Nav2 lifecycle, planner, controller, map server, Smac Hybrid-A*, and the
Autoware `Control` message type as a simulation command boundary. Install the
build tools and runtime package set:

```bash
sudo apt update
sudo apt install \
  build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-autoware-control-msgs
```

Initialize rosdep if this machine has not used it before, then resolve the
package manifest dependencies:

```bash
sudo rosdep init  # only when rosdep reports it is not initialized
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/atlas_nav2_demo --ignore-src -r -y
```

No Autoware source checkout is needed for this isolated demo. The package uses
the installed message package and does not launch or modify the Autoware stack.

## Build and test

The default overlay and build directory are placed in the ignored
`.nav2_cache/ackermann/` directory under the development checkout. To choose an
external location, set `ATLAS_NAV2_OVERLAY` to an absolute install-prefix path
before running the campaign.

```bash
source /opt/ros/humble/setup.bash
export ATLAS_NAV2_OVERLAY="$PWD/.nav2_cache/ackermann/install"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --base-paths src/atlas_nav2_demo --packages-select atlas_nav2_demo \
  --build-base "$(dirname "$ATLAS_NAV2_OVERLAY")/build" \
  --install-base "$ATLAS_NAV2_OVERLAY" --executor sequential \
  --cmake-args -DBUILD_TESTING=ON
source "$ATLAS_NAV2_OVERLAY/setup.bash"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon test \
  --base-paths src/atlas_nav2_demo --packages-select atlas_nav2_demo \
  --build-base "$(dirname "$ATLAS_NAV2_OVERLAY")/build" \
  --install-base "$ATLAS_NAV2_OVERLAY" --executor sequential
colcon test-result --test-result-base \
  "$(dirname "$ATLAS_NAV2_OVERLAY")/build/atlas_nav2_demo" --verbose
ros2 pkg prefix atlas_nav2_demo
```

The package prefix must be `$ATLAS_NAV2_OVERLAY/atlas_nav2_demo`. Both
acceptance runners fail closed if the package resolves from an underlay rather
than the first sourced colcon overlay.

## Run the complete headless campaign

Start from a clean terminal with no other Atlas or Nav2 test graph running.
The entry point builds/tests the package, checks overlay precedence, then runs
straight 5x, curved 5x, avoidable obstacle 5x, blocked route 1x, all six
full-graph safety faults, and 20 sequential cancellation trials. It uses
`ROS_LOCALHOST_ONLY=1`, isolated sequential ROS domains, and does not start
RViz. It keeps going only while each preceding check passes.

```bash
source /opt/ros/humble/setup.bash
tools/nav2/run_ackermann_native_validation.sh
```

Optional environment settings:

```bash
export ATLAS_NAV2_OVERLAY="$HOME/ros_overlays/atlas_ackermann/install"
export ATLAS_VALIDATION_EVIDENCE_ROOT="$HOME/validation_evidence/ackermann"
export ATLAS_NAV2_DOMAIN_START=20
tools/nav2/run_ackermann_native_validation.sh
```

The domain start reserves 42 consecutive IDs: four navigation scenario groups,
the six-fault run, and the twenty cancellation trials. Choose a range unused on
the machine. The script saves evidence in a UTC-named subdirectory of
`ATLAS_VALIDATION_EVIDENCE_ROOT` (default: `~/validation_evidence/ackermann`).
Build/test logs and runtime data go there, not into source directories.

To run an individual scenario after building and sourcing the overlay:

```bash
export ROS_LOCALHOST_ONLY=1
ros2 run atlas_nav2_demo run_ackermann_acceptance.py \
  --scenario avoidable --repetitions 5 --domain-start 40 \
  --evidence-root "$HOME/validation_evidence/ackermann/avoidable"
```

The cancellation-only campaign, with host/process resource sampling, can be
run independently:

```bash
python3 tools/nav2/run_ackermann_timing_campaign.py \
  --trials 20 --domain-start 80 \
  --evidence-root "$HOME/validation_evidence/ackermann/cancellation"
```

## Resource and scheduling measurements

The campaign continuously records host CPU busy percentage, load averages,
available and swap memory, swap-in/out counters, and Linux PSI data when
available. For matching Nav2 processes it records per-process CPU percentage,
RSS, `/proc/<pid>/schedstat` runtime/run-queue wait/timeslices, and voluntary
and involuntary context-switch counts. These are sampled at 0.5 s; short-lived
events between samples can be missed. Evidence is JSONL and contains monotonic
sample timestamps.

For an independent lightweight cross-check, install `sysstat` and run these
in another terminal during a focused trial:

```bash
sudo apt install sysstat
vmstat 1
mpstat -P ALL 1
pidstat -urdw -p ALL 1
free -h
```

Stop the foreground campaign with Ctrl+C only when necessary; the script
terminates its sampler and preserves partial evidence. Do not run RViz or
another simulation graph concurrently. A run interruption should be reported
with its source monotonic timer stamps, host samples, and per-process scheduler
counters. Do not infer CPU saturation from one process reaching one full core.

## Pass/fail criteria

- Build and package tests complete successfully; report the exact test count.
- The package prefix resolves from the selected development overlay.
- Straight, curved, and avoidable scenarios each have five successful goals;
  blocked route aborts safely. A safe abort is never counted as goal success.
- Navigation samples show no adapter rejection, collision clamp, safety fault,
  or configured steering/speed/acceleration/deceleration violation.
- All six full-graph fault scenarios pass their stated safe-stop/rearm checks.
- Every cancellation trial publishes gate safe-zero and applies the simulator
  zero target within 250 ms; the modeled stopping envelope is respected.
- Preserve any failed run. Finite passing trials do not establish a hard
  real-time guarantee. A reproduced common scheduling stall keeps timing
  acceptance open, even if nominal cancellation timing passes.

## Remaining safety limitation

The controller, gate, adapter, and simulator are separate processes, but share
the same host scheduler. During a period when a watchdog process is not
scheduled, its software timeout cannot publish a timely command. The observed
gate/simulator timer gaps therefore remain a timing acceptance issue. A future
physical vehicle interface must enforce a command timeout independently at the
actuator or vehicle-control electronics and provide a suitably independent,
hardware emergency-stop circuit. A second ROS process is not hardware-level
safety independence. This repository does not implement physical motor control
and no hardware safety claim is made.
