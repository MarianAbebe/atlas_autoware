# Reviewed Atlas demo milestones

These are isolated experiments, not production Atlas configuration. The
per-package validation reports distinguish measured passes, failures and
unverified work. No physical vehicle test or verified map registration is claimed.

| Package | Preserved milestone | Qualification |
| --- | --- | --- |
| atlas_lidar_perception_test | Synthetic LiDAR preprocessing and pinned CPU clustering test | Synthetic geometry; no physical detection performance claim |
| atlas_map_integration_demo | Original campus map loading and routing, 15 checks | Mission planner not executed; rendered images unverified |
| atlas_trajectory_generation_demo | Pinned planning and smoothing, 35 checks | Static ego fixture; spatial velocity profile, not executed control |
| atlas_vehicle_motion_demo | Bounded pinned simulator motion bench | Scripted bench commands; loose stop tolerance permits residual drift |
| atlas_closed_loop_demo | Real-map closed-loop drive, 38 checks; six safety cases, 41 each | Simulation only; provisional dynamics; faults injected near 0.5 m/s; rendering unverified |

The existing unpushed NDT milestone (`539d487`) was also reviewed before this
preservation commit. Its source and synthetic localization test are separate
from the campus closed-loop experiment.

## Evidence and portability

Generated maps, point clouds, recordings, logs, plots, build/install trees and
temporary dependency selections are excluded. Original datasets remain outside
the repository; upstream Autoware sources are not vendored. Closed-loop numerical
thresholds, the final summary, pinned revisions and hashes of the local final
recordings are retained under `src/atlas_closed_loop_demo/results/`.
Historical report links to raw evidence require the original local recordings.
They do not imply those recordings are included in a clone.

Scripts preserve the validated workstation defaults: repository
`/home/marian/roam/atlas_autoware`, pinned source
`/home/marian/roam/references/autoware`, and original maps under
`/home/marian/Downloads`. Launch arguments expose map filenames where supported.
Older map, trajectory, perception and motion instructions reference `/tmp`
overlays which may disappear after reboot. Their previous runtime results are
historical, not proof that those caches still exist. The closed-loop build uses
the durable ignored `.closed_loop_cache` and audits dependencies before building.
Review paths and provide the original inputs before reproducing on another host;
these demos are not a relocatable binary distribution. No path relocation or
controller behavior was changed during preservation.

## Fast preservation checks

From the repository root:

```bash
python3 scripts/check_demo_milestones.py
source /opt/ros/humble/setup.bash
python3 src/atlas_closed_loop_demo/scripts/dds_probe.py --results /tmp/atlas_milestone_dds
git diff --cached --check
```

The first command checks Python/shell syntax, XML/YAML configuration, preserved
threshold consistency and, when available, all hashed local final evidence. DDS
checks isolated String delivery only. Neither command reruns the campus drive or
the six motion safety cases. Full reproduction commands and their limitations
remain in each package's README and VALIDATION.md.

Preservation review on 2026-10-20: Python/shell syntax, five package manifests,
13 YAML/RViz configurations, all 217 local evidence hashes, and staged whitespace
checks passed. The isolated DDS probe delivered 117 messages. The saved map and
trajectory reports contain 15/15 and 35/35 passes; final closed-loop records
contain 38/38 plus six sets of 41/41. Independently summing raw odometry gives
69.78412163252122 m, matching the preserved summary. Original map SHA-256 values
still match the final recording; pinned upstream working trees are clean.
Full motion scenarios and C++ builds were not rerun during preservation. Older
perception/motion `/tmp` install caches are absent in this session.
