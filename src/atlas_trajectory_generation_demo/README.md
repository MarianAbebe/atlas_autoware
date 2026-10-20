# Real-map trajectory generation demo

This isolated demo reuses the original campus map loaders and selects the actual
route `38184 -> 23892` from the main routing component. It excludes the disconnected
loading dock. The shorter route is approximately 69.779 m, wholly inside the PCD
XY bounding box; this does not certify dense scan coverage or registration.

Executed and validated pipeline:

```text
loaded Lanelet2 + actual LaneletRoute + synthetic ego snapshot
  -> pinned autoware_path_generator (PathWithLaneId)
  -> pinned motion_utils::convertToPath adapter (drop lane IDs only)
  -> pinned autoware_planning_topic_converter (Trajectory)
  -> pinned autoware_velocity_smoother / Analytical (bounded velocity profile)
```

The tiny adapter preserves all point poses, bounds, velocities, and timestamps.
It generates neither a path nor a trajectory/profile. It bridges the pinned
generator's `PathWithLaneId` output to the pinned converter's `Path` input without
adding an unnecessary behavior-planning/perception stack for this obstacle-free
geometry/profile experiment.

The full run passed 35 bounded checks across 20 smoother publications. The final
trajectory has 226 points over 70.279 m, with speeds between 0 and 1 m/s and a
terminal stop. Point `time_from_start` values are zero: this is a spatial velocity
profile, not an absolute time schedule. See [VALIDATION.md](VALIDATION.md) for
measurements, limitations, and saved runtime evidence.

The feedback fixture repeats a **static planning snapshot**, not a physically
integrated vehicle: pose at the computed map-frame route start, speed 0.2 m/s,
acceleration zero, control disabled. It publishes current timestamps and a
clearly named `atlas_trajectory_ego` TF/marker. Controller and simulator are absent.
Synthetic vehicle geometry is copied in intent from the prior test, not measured
Atlas parameters. Experimental final limits are 1 m/s and +/-0.5 m/s².
No road speed-limit tag or map coordinate is changed.

Build only after the grouped system dependency audit is satisfied:

```bash
bash src/atlas_trajectory_generation_demo/scripts/build_demo.sh
```

The script checks pinned commits, audits all dependencies, reuses available
overlays, and never invokes an installer. Cached builds stay under `/tmp`.

For the full bounded runtime validation after building:

```bash
source /opt/ros/humble/setup.bash
source /tmp/atlas_map_upstream_install/setup.bash
source /tmp/atlas_map_demo_install/setup.bash
source /tmp/atlas_trajectory_upstream_install/setup.bash
source /tmp/atlas_trajectory_demo_install/setup.bash
export ROS_DOMAIN_ID=78
python3 src/atlas_trajectory_generation_demo/scripts/run_validation.py
```

`--route-only` deliberately disables the planning components and validates only
the route, fixtures, original map publications, and RViz route display.
`--no-rviz` skips GUI validation. Successful route-only results are **not**
trajectory or velocity-smoothing successes.

For an interactive full launch, use the same environment and:

```bash
ros2 launch atlas_trajectory_generation_demo trajectory_demo.launch.py
```

The full launch contains no controller, simulator, physical vehicle interface,
or guessed alignment correction. RViz's fixed frame is `map`. No screenshot
capture is attempted. Results and failures are saved per run in
`/tmp/atlas_trajectory_results`; see [VALIDATION.md](VALIDATION.md) for the actual
executed stages and outstanding blockers.
