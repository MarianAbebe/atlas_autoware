# Native Ubuntu 22.04 Test Preparation

**Release branch:** `handoff/native-ubuntu-validation-2026-10-25`
**Scope:** synthetic simulation only. Native x86_64 support is a target, not yet
experimentally verified. This guide does not authorize physical vehicle operation
or autonomous engagement.

## Supported target and prerequisites

- Ubuntu 22.04 LTS (Jammy), ROS 2 Humble, Python 3.10, x86_64.
- Recommended: 8 GiB RAM, 6 or more CPU cores, and at least 50 GiB free disk
  before importing/building the pinned Autoware workspace. A GPU/OpenGL driver is
  recommended for RViz; the baseline can be run headless.
- This repository was developed on Ubuntu 22.04 ARM64 in a 4 GiB UTM VM. That
  validates source portability only partially; it does not validate x86_64 ABI,
  native performance, or the clean-machine build.
- Ubuntu 24.04/Jazzy is unsupported for this release. Do not substitute ROS
  distributions or upgrade pinned dependencies silently.

## 1. Obtain the release source

The release branch currently exists locally and has not been pushed. A team member
must first receive it through the team's approved Git remote or a verified source
bundle. Once available remotely:

```bash
git clone <approved-atlas-repository-url> atlas_autoware
cd atlas_autoware
git checkout handoff/native-ubuntu-validation-2026-10-25
```

The branch name is exact. Verify `git status --short` is clean before following
the rest of the guide.

## 2. Install reviewed system prerequisites

Install Ubuntu 22.04 and ROS 2 Humble using the official ROS Humble installation
instructions. The project does not run privileged installation commands.
Required command-line tools include Git, CMake, colcon, rosdep, vcstool,
`build-essential`, `binutils-gold` (`ld.gold`; existing Atlas colcon scripts pass
`-fuse-ld=gold`), and `python3-catkin-pkg`. The ROS dependency closure includes
packages declared by the pinned Autoware and Atlas manifests. Installing those
system packages requires `sudo` and operator review; this repository does not
run privileged installers.

After installing ROS Humble and the standard build tools, open a new shell and
source ROS before running preflight:

```bash
source /opt/ros/humble/setup.bash
cd ~/atlas_autoware
tools/native_ubuntu/preflight.sh
```

Review the exact dependency closure command without installing anything:

```bash
source /opt/ros/humble/setup.bash
ATLAS_AUTOWARE_ROOT="$HOME/roam/references/autoware" \
  python3 tools/native_ubuntu/dependency_plan.py
```

The script prints a `rosdep install` command covering the Atlas packages and
their selected pinned source dependencies. Review it, then run the printed
command manually. If rosdep has not been initialized on this Ubuntu install,
follow the ROS 2 Humble rosdep setup instructions first. No unpinned package
upgrade is part of the Atlas workflow.

## 3. Import and verify pinned Autoware 1.9.0

By default, Autoware is stored adjacent to the clone at
`../references/autoware`. To choose another location, export
`ATLAS_AUTOWARE_ROOT` before each tool invocation or in the current shell:

```bash
export ATLAS_AUTOWARE_ROOT="$HOME/roam/references/autoware"
tools/native_ubuntu/fetch_autoware.sh
```

The helper checks out/imports the pinned repositories and verifies every recorded
revision in `tools/native_ubuntu/autoware_pins.tsv`. It stops on a revision
mismatch. It requires network access and `vcs` (`python3-vcstool`).

## 4. Transfer campus map assets

Three map assets are external to Git. Transfer the exact files through the team's
approved file-transfer method into one directory on the target laptop:

```text
finalMap_repaired.osm
map_projector_info.yaml
pointcloud_map_georeferenced.pcd
```

The SHA-256 values recorded for the validated files are:

| File | SHA-256 |
|---|---|
| `finalMap_repaired.osm` | `1e9360ea62b709271007d27672120b3d0040ebabc6667d646de68734b1702fee` |
| `map_projector_info.yaml` | `614af74d6aaefe052c435338b585797a451e7922efc290329374c2abd2b8992d` |
| `pointcloud_map_georeferenced.pcd` | `6121b6bf8df4324ab42286822f0e6f00c67ecc10daecf84d180129a6069ef9a3` |

Set the asset directory explicitly; do not rely on a user's Downloads folder:

```bash
export ATLAS_MAP_DIR="$HOME/atlas_assets"
(cd "$ATLAS_MAP_DIR" && sha256sum -c \
  "$HOME/atlas_autoware/tools/native_ubuntu/map_assets.sha256")
```

All three hashes must report `OK`. The assets are not bundled in Git. Their
validated source files were under `/home/marian/Downloads`; they need approved
transfer to the target machine.

## 5. Preflight and build

Preflight checks OS, architecture, RAM, disk, ROS/tools, Python, OpenGL
availability, pinned Autoware root revision, and map-file hashes:

```bash
source /opt/ros/humble/setup.bash
cd ~/atlas_autoware
export ATLAS_AUTOWARE_ROOT="$HOME/roam/references/autoware"
export ATLAS_MAP_DIR="$HOME/atlas_assets"
tools/native_ubuntu/preflight.sh
```

Warnings are actionable (for example, lower RAM or no OpenGL inspection tool).
Resolve every `MISSING` item before building. The build script uses existing
package build workflows, limits parallel compilation to one job for memory
safety, imports/verifies pinned Autoware, builds required Atlas packages, and
runs a no-graph package/asset smoke test:

```bash
tools/native_ubuntu/build.sh
```

This is a clean-source build on a new clone, but it can consume substantial disk
and time. Do not begin unless preflight reports adequate space. No native x86_64
build has yet been run; a successful build on Vyapak's machine must be recorded
as new evidence.

## 6. Environment and no-graph smoke test

For each new terminal:

```bash
cd ~/atlas_autoware
export ATLAS_AUTOWARE_ROOT="$HOME/roam/references/autoware"
export ATLAS_MAP_DIR="$HOME/atlas_assets"
source src/atlas_full_stack_demo/scripts/environment.sh
tools/native_ubuntu/smoke_test.sh
```

`smoke_test.sh` verifies package discovery, installed launch/config files,
installed executable permissions, Python syntax for the main scene and
localization initializer, and map checksums. It does not start ROS nodes.

## 7. Safe stationary validation (synthetic simulation only)

Before starting, close other Atlas/Autoware graphs and RViz windows. The runner
also blocks when it detects existing Atlas processes or RViz. Use a unique output
directory; by default it creates one under `/tmp` with a UTC timestamp:

```bash
cd ~/atlas_autoware
source /opt/ros/humble/setup.bash
export ATLAS_AUTOWARE_ROOT="$HOME/roam/references/autoware"
export ATLAS_MAP_DIR="$HOME/atlas_assets"
source src/atlas_full_stack_demo/scripts/environment.sh
python3 tools/native_ubuntu/validate_stationary.py
```

The runner starts the diagnostic recorder and scheduler sampler, launches the
synthetic full stack with `request_engagement:=false`, initializes localization
through `initialize_localization.py`, then requires both watchdogs to arm and
remain healthy for at least 120 seconds. It verifies map-to-base TF, NDT/EKF
outputs, simulator speed, scene scan skips, and that no positive Engage message
was observed. Only after that phase passes does it start exactly one
`full_stack_lightweight.rviz` for a further 120 seconds while monitoring safety.
It never runs `manual_start.py`, never publishes Engage, and never connects
physical actuator interfaces. `--without-rviz` omits the visualization phase;
it does not shorten the 120-second stationary check. `--hold-sec N` accepts
values only at or above 120.

Each invocation creates `summary.json`, launch/initializer/RViz logs, recorder
output, scheduler telemetry, and map checksum results under its unique output
directory. A watchdog latch, stale status, scan skip, unexpected engagement, motion, or
process failure marks the run `FAIL`. Missing required telemetry (odometry, TF,
NDT/EKF, or watchdogs that never arm) is `INCONCLUSIVE`, not `PASS`. The runner
never publishes `Engage` and never calls watchdog rearm. On Ctrl+C, it sends
SIGINT only to process groups it started and allows rosbag shutdown before
finalizing the summary. Watchdog status history is written to
`watchdog_history.json`. Preserve failed-run directories unchanged.

## Expected result and interpreting it

A passing run means the synthetic, stationary graph satisfied the checks above
for one run. It is not evidence of repeatability, engaged driving, obstacle-stop
performance, localization accuracy against a real independent reference, or
hardware readiness. Three consecutive passes are recommended before treating
stationary behavior as reproducible. A single pass does not establish native
platform performance.

## Optional visualization stress test

Not part of the baseline and must not be automated: only after a passing
stationary baseline, an operator may separately test additional RViz displays.
Use one RViz process and continuously observe both watchdogs. Stop at the first
stale input or latch, preserve logs, and do not reset/rearm as part of the same
run. Do not use the original heavyweight configuration as the baseline.

## Clean shutdown

Press Ctrl+C once in the validation terminal. Wait for the summary and bag
finalization. If the helper exits unexpectedly, inspect only the output directory
and process IDs it recorded before manually stopping its own leftover children.
Do not use broad `pkill` commands or terminate unrelated ROS processes.

## Known limitations

- x86_64 native build and runtime remain to be tested.
- Map assets are external and require exact-hash transfer.
- The NDT localization map used by this full-stack demonstration is synthetic;
  the actual campus PCD is loaded for map visualization/integration but is not a
  validated real-campus NDT map.
- Static stationary validation is distinct from engaged closed-loop simulation.
- Obstacle stops, localization-loss stopping, long-duration stability, and
  physical hardware remain separate validation tasks.
- The 300 ms watchdog contracts remain unchanged. Earlier synchronized runs
  captured graph-wide scheduling interruption; investigate and retain any new
  evidence rather than weakening freshness behavior.
