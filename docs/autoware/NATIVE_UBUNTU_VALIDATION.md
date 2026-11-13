# Native Ubuntu Autoware Ackermann validation

This procedure is for a dedicated Ubuntu 22.04 x86_64 development machine. It
does not install packages, change clock services, modify partitions, or start a
physical vehicle. The ARM64 Mac VM has not been treated as equivalent native
validation, and native execution must not be assumed to eliminate the observed
timing interruptions.

## Prerequisites and inputs

- Ubuntu 22.04 x86_64 with ROS 2 Humble under `/opt/ros/humble`.
- `build-essential`, CMake, Git, Python 3.10, `python3-colcon-common-extensions`,
  `python3-rosdep`, `vcstool`, `binutils-gold`, and the ROS development tools
  already available on the selected Autoware install.
- A clean development checkout on `feature/prd-gap-closure` and its nested
  Autoware source checkout at the revisions in
  [`tools/native_ubuntu/autoware_pins.tsv`](../../tools/native_ubuntu/autoware_pins.tsv).
- An Autoware 1.9.0 dependency overlay. Set `ATLAS_AUTOWARE_SETUP` to its
  `install/setup.bash` and `ATLAS_AUTOWARE_ROOT` to the matching source root.
- The three map files `finalMap_repaired.osm`, `map_projector_info.yaml`, and
  `pointcloud_map_georeferenced.pcd` in `ATLAS_MAP_DIR` (default
  `~/atlas_assets`). Verify the supplied SHA-256 manifest before use.
- Adequate disk for the pinned Autoware checkout and build. The existing
  `tools/native_ubuntu/preflight.sh` recommends 50 GiB free and 8 GiB RAM for a
  clean source build; these are planning recommendations, not hard runtime
  limits. No full Autoware rebuild is required when a matching installed
  dependency overlay is available.

For the base build tools, the expected Ubuntu package names are:

```bash
sudo apt install build-essential cmake git python3-colcon-common-extensions \
  python3-rosdep python3-vcstool binutils-gold python3-numpy python3-pytest
```

Install ROS 2 Humble from the official ROS Ubuntu Jammy instructions. Resolve
the pinned Autoware source dependencies with `rosdep` after checking out the
matching repositories, for example:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths "$ATLAS_AUTOWARE_ROOT/src" \
  src/atlas_vehicle_interface src/atlas_map_integration_demo \
  src/atlas_closed_loop_demo src/atlas_trajectory_generation_demo \
  src/atlas_obstacle_stopping_demo src/atlas_perception_planning_demo \
  src/atlas_localization_closed_loop_demo src/atlas_full_stack_demo \
  --ignore-src -r
```

Review the dependency plan before approving system package changes. The native
runner itself never invokes apt or rosdep.

The repository includes `tools/native_ubuntu/fetch_autoware.sh` and
`build.sh` for provisioning the pinned reference checkout, but those scripts
perform network cloning and builds. Review and run them explicitly when needed;
the validation runner below never installs dependencies or fetches sources.

## Checkout and environment

```bash
git clone --branch feature/prd-gap-closure --single-branch \
  https://github.com/MarianAbebe/atlas_autoware.git atlas_autoware_dev
cd atlas_autoware_dev
git submodule update --init --recursive  # only if `git submodule status` lists entries
git status --short --branch
```

Keep the protected stable checkout separate. Do not copy its `build`, `install`,
or `log` directories. Configure paths for the native machine:

```bash
export ATLAS_AUTOWARE_ROOT="$HOME/roam/references/autoware"
export ATLAS_AUTOWARE_SETUP="$HOME/roam/overlays/autoware_1_9/install/setup.bash"
export ATLAS_MAP_DIR="$HOME/atlas_assets"
export ATLAS_AUTOWARE_OVERLAY="$HOME/roam/overlays/atlas_autoware_native"
export ATLAS_VALIDATION_EVIDENCE_ROOT="$HOME/validation_evidence/autoware_ackermann"
export ATLAS_VALIDATION_DOMAIN_START=120
```

The expected source order is ROS Humble, the matching Autoware dependency
overlay, the Atlas Euclidean cluster overlay built by the runner, then the
Atlas integration overlay. Do not source a different Autoware build after the
Atlas overlay. The runner checks the selected package prefix after build.

## Commands

Start with a non-mutating preflight:

```bash
tools/autoware/run_ackermann_native_validation.sh --check
```

The runner supports the following modes:

```bash
tools/autoware/run_ackermann_native_validation.sh --build
tools/autoware/run_ackermann_native_validation.sh --smoke
tools/autoware/run_ackermann_native_validation.sh --faults
tools/autoware/run_ackermann_native_validation.sh --repeatability
tools/autoware/run_ackermann_native_validation.sh --all
```

`--build` makes an isolated source copy of the pinned Simple Planning Simulator
with optional integration-step telemetry (the pinned source checkout is never
edited), and compiles it with the Atlas vehicle, map, route, trajectory,
obstacle, perception, localization, and full-stack packages, plus the pinned
Euclidean cluster package and Atlas sensor-format/distortion adapters
(`atlas_sensor_kit`, `atlas_distortion_corrector_test`). Compilation is
sequential (`-j1`) into the external
overlay path. The native lap runner enables simulator-step telemetry; manual
recorder runs require `--steering-step-diagnostics` explicitly. `--smoke` runs
one bounded full-stack route. `--faults` runs the
currently available full-graph E-stop/explicit-clear/no-route-resumption test,
synthetic source-loss stop, and NDT-deactivation stop. These cover two
localization/sensor-loss paths but do not constitute the complete fault matrix.
`--repeatability` runs five
separate one-lap trials and one continuous five-lap trial. Each trial runs
sequentially on its own ROS domain and writes to a new evidence directory.
`--all` executes build, smoke, E-stop, and repeatability in that order and
stops on failure. Each individual mode can be run without rebuilding if the
matching overlay already exists.

The runner uses `ROS_LOCALHOST_ONLY=1`, does not start RViz, bounds each trial
with a timeout, and preserves launch logs/results. It exits nonzero on a
failed test and will not overwrite an existing evidence directory. Select an
unused domain range; the runner accepts starts from 1 through 222.

For lower-level localization fault checks, the existing
`src/atlas_localization_closed_loop_demo/scripts/run_validation.py` supports
`nominal`, `drop_ndt`, `drop_ekf`, `no_ndt`, and `no_ekf`. Those are localization
demo tests, not substitutes for full Autoware-graph safety evidence.

## Monitoring and evidence

During a campaign, a second terminal may collect host/process observations:

```bash
vmstat 1
mpstat -P ALL 1
pidstat -urdw -p ALL 1
free -h
```

Keep runs sequential and do not start RViz. The route recorder reports source
header stamps, monotonic local receipt times, ROS-time age, route tracking,
steering, simulator integration-step rate where the instrumented simulator is
present, and watchdog state. For detailed perception-stage receipt tracing,
run `run_lap_validation.py --trace-perception`; this adds five best-effort,
depth-one PointCloud2 observers for intermediate clouds. The raw sensor
observer remains enabled as the source-age baseline. Reserve the additional
observers for diagnostic trials. See the `perception_trace_20261113` evidence and
`ACKERMANN_INTEGRATION.md` for known source-age failures and their limits.

Evidence is written under `ATLAS_VALIDATION_EVIDENCE_ROOT` (default
`~/validation_evidence/autoware_ackermann_native`). Preserve failed runs.
Do not move evidence into the repository or report a native pass until the
runner has actually completed on native x86_64 Ubuntu.

## Pass criteria and known limitations

- `--check` verifies host architecture, ROS setup, key commands, Autoware
  overlay, source root, and required map assets without installing or building.
- Build passes only if the Atlas integration overlay resolves first for
  `atlas_full_stack_demo`.
- Smoke/repeatability require the runner's route acceptance result, no safety
  latch, and valid configured kinematic checks. A goal reached after a latch is
  not a pass.
- Fault mode covers full-graph E-stop, explicit clear, latching, no previous
  route resumption, synthetic source loss, and NDT deactivation while the
  scene continues. Command loss, frozen odometry, stale predicted objects,
  invalid commands, cancellation, command-owner conflict, and clock
  discontinuity are not implemented as full-graph runner cases and remain
  open.
- Finite runs do not prove a hard real-time bound. The software gate and
  simulator remain subject to a shared OS scheduler. A physical interface
  needs an independently enforced actuator command timeout and an appropriate
  hardware E-stop architecture. No motor driver or physical validation is
  supplied here.
- Localization in the route showcase uses synthetic scan/map/twist inputs.
  Successful route following does not validate registration against the actual
  campus point-cloud map or real sensor localization.

If prerequisites are missing, install them only through the machine's normal
approved package-management process. Do not change NTP, PTP, guest clock
synchronization, freshness limits, or safety behavior to make a test pass.
