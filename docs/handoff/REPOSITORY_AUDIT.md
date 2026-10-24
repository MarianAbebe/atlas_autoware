# Repository readiness audit

Audit performed read-only on 2026-10-23 UTC. No files were deleted, committed or
pushed, and no ROS graph was launched.

## Release preparation update (2026-10-24)

The audit below is a snapshot from before the native-Ubuntu preparation. The
release branch now includes portable `ATLAS_AUTOWARE_ROOT` and `ATLAS_MAP_DIR`
resolution for the full-stack and related launch/validation paths, pinned-source
import/revision verification, preflight, dependency-plan, build and no-graph
smoke-test tools under `tools/native_ubuntu/`. The old overlay caches are no
longer a documented prerequisite for a clean build. The three campus map assets
remain external and must be transferred and hash-verified. A clean Ubuntu 22.04
x86_64 build has not been performed; this remains the primary reproduction
blocker. The release branch is local until transferred or explicitly pushed.

## Critical reproducibility blockers

1. **The working implementation is not committed.** Full untracked-file expansion
   reported 2,954 worktree entries: 19 modified and 2,935 untracked. The complete
   `src/atlas_full_stack_demo`, localization, obstacle and perception demo
   packages are untracked. A fresh clone does not contain the validated state.
2. **No clean-machine dependency bootstrap is validated.** The full-stack
   environment directly sources machine-local `.closed_loop_cache`,
   `.obstacle_cache`, `.perception_cache`, and `.localization_cache` overlays.
   Together with `.full_stack_cache`, they occupy roughly 948 MiB on this VM.
3. **Hardcoded local paths remain.** Map launch defaults use
   `/home/marian/Downloads`; several lower-level build/launch scripts use
   `/home/marian/roam/...`. The full-stack build assumes Autoware at
   `$HOME/roam/references/autoware`. A different username or layout will fail.
4. **The map inputs are external and not versioned in this repository.** Their
   hashes are documented, but another developer must receive them separately.
5. **Primary evidence is in `/tmp`.** Both the latest synchronized recording and
   previous fault recording are vulnerable to VM cleanup and are not part of the
   repository.

These blockers prevent claiming that another machine can reproduce the system
from a clone today. The current reference VM remains runnable.

## Pinned upstream state

The established build checks these exact clean tracked revisions:

| Checkout | Revision |
|---|---|
| Autoware root | `10718787ba6e28f038a0cb29ff99cc627b5abfd2` |
| `autoware_core` | `f25f83c632c1984ec276c894c41857d4abc0dad8` |
| `autoware_universe` | `6e477c645efec33f7909095eea684474e97f5e3d` |
| simple planning simulator | `82d2c93a28a25f8802e8f5a21b4ec850beea9080` |

No pinned source modification was found by the established build's tracked-file
check.

## Dependencies and build artifacts

All setup files required by `environment.sh` exist on the current VM. The package
declares its direct ROS/Python runtime dependencies, including localization/map/
perception demo packages, messages, `rclpy`, `sensor_msgs_py`, NumPy, rosbag,
ground filter and components. The installed transitive Autoware dependency set is
provided by the local caches rather than a portable full-stack manifest.

Current cache sizes:

| Cache | Size |
|---|---:|
| `.closed_loop_cache` | 778 MiB |
| `.obstacle_cache` | 53 MiB |
| `.perception_cache` | 44 MiB |
| `.localization_cache` | 55 MiB |
| `.full_stack_cache` | 18 MiB |

`.localization_cache` and `.full_stack_cache` are ignored at the repository root.
Other caches use their own local ignore handling, but the overall clean-build and
artifact policy needs consolidation before transfer.

## Installed package and permissions

`ros2 pkg prefix atlas_full_stack_demo` resolves to:

```text
/home/marian/roam/atlas_autoware/.full_stack_cache/install/atlas_full_stack_demo
```

Installed executables were verified for scene, object adapter, manual start,
localization-only initialization, diagnostic recording and scheduler telemetry.
The installed files are executable.

Several source scripts are mode `0664`, including `build_demo.sh`,
`environment.sh`, `manual_start.py` and `scene.py`. This is currently intentional
in usage: shell scripts are invoked with `bash`, Python utilities with `python3`,
and CMake `install(PROGRAMS)` gives runtime copies executable permissions. It is
still a packaging inconsistency worth cleaning after the handoff.

Generated `__pycache__` directories are present and untracked. They are not a
runtime blocker but should not be included in a future reviewed commit.

## Documentation and test evidence

- Existing implementation details: `src/atlas_full_stack_demo/README.md`.
- Earlier integrated results: `docs/full_stack/REPORT.md` and
  `docs/full_stack/validation/summary.json`.
- Latest synchronized evidence: `/tmp/atlas_stability_20261023_213504`.
- Previous fault: `/tmp/atlas_trajectory_diagnostic_01/timing_bag`.

The established full-stack build was actually run on 2026-10-23 after scheduler
telemetry integration. It completed and reported four existing static-object
tests and three scheduler tests passing. Documentation syntax/link inspection in
this handoff does not constitute a new runtime validation.

## Required repository work before cross-machine reproduction

1. Review and commit the intended source/doc changes as a coherent revision.
2. Remove generated files from the proposed commit and preserve unrelated user
   changes.
3. Replace or parameterize hardcoded user paths.
4. Create a versioned dependency/build manifest for all required Autoware
   packages instead of relying on existing caches.
5. Define a controlled way to distribute and verify the three map assets.
6. Copy diagnostic evidence from `/tmp` to durable team storage without changing
   the original bags.
7. Perform the P0 clean-machine stationary reproduction from
   [CONTINUATION_PLAN.md](CONTINUATION_PLAN.md).
