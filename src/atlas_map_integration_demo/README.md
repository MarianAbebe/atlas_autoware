# Atlas campus map integration demo

This test-only package loads the original campus maps through pinned Autoware
components and validates a real Lanelet2 route. It does not run the mission
planner node, generate a timed trajectory, control a vehicle, or modify maps.

The routing probe consumes the loader's published `LaneletMapBin`. It uses the
same `autoware`/Vehicle rules and graph-construction utility as the pinned core
route handler. It computes weak and strong connected components, selects a
successor-only route in the largest weak component, verifies every transition,
and publishes `LaneletRoute`, `Path`, and diagnostic markers. The PCD is published
unchanged. No alignment transform or artificial map-to-robot TF is broadcast.

Builds live in `/tmp/atlas_map_upstream_*` and `/tmp/atlas_map_demo_*`; source stays
in this package. The pinned upstream checkout is never edited.

To audit dependencies and build only missing pinned packages (never installing
system packages), run:

```bash
bash src/atlas_map_integration_demo/scripts/build_demo.sh
```

After building, launch an interactive map-only demonstration:

```bash
source /opt/ros/humble/setup.bash
source /tmp/atlas_map_upstream_install/setup.bash
source /tmp/atlas_map_demo_install/setup.bash
export ROS_DOMAIN_ID=77
ros2 launch atlas_map_integration_demo campus_map.launch.py
```

Stop it with Ctrl-C. The launch stays alive for viewing; the separate validator
starts it, observes all publications and RViz subscriptions, waits eight seconds
with nodes alive, then shuts it down:

```bash
python3 src/atlas_map_integration_demo/scripts/run_validation.py
```

`--no-rviz` validates only the map/routing stage. Results go to a fresh
`/tmp/atlas_map_demo_results/run_YYYYMMDD_HHMMSS` directory. The validator exits
nonzero for missing publications, invalid payloads/frames, dead nodes, missing
RViz subscriptions, or critical runtime errors. It checks every published XYZ
coordinate against the original PCD and checks source SHA-256 hashes before and
after the run. It deliberately does not attempt screenshots.

Read-only alignment diagnostics consume actual loader-projected centerlines:

```bash
python3 src/atlas_map_integration_demo/scripts/analyze_alignment.py /tmp/atlas_map_demo_results/run_20261012_022357
```

The reported occupancy distances and lower-envelope elevations are diagnostics,
not verified feature correspondences or an estimated transform. See
[VALIDATION.md](VALIDATION.md) for results, limitations, and driving blockers.
