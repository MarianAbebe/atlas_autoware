# ROAM Atlas — campus map integration and routing validation

Executed 2026-10-12 UTC on Ubuntu 22.04 ARM64 / ROS 2 Humble.

**Result:** original Lanelet2 and PCD maps loaded through pinned Autoware;
a real vehicle route was computed, verified, and published; RViz launched and
subscribed to every configured display. The bounded runtime validation passed
all 15 checks and exited 0. This is a map/routing milestone, not closed-loop
driving, localization on the real cloud, or a calibrated alignment milestone.

No upstream source, production Atlas configuration, or original dataset was
changed. No packages were installed, commits created, or changes pushed. No map
repair, coordinate transform, elevation correction, or screenshot was attempted.

## 1. Files and build scope

Original datasets:

| Path | Bytes | SHA-256 |
|---|---:|---|
| `/home/marian/Downloads/finalMap_repaired.osm` | 20,084,751 | `1e9360ea62b709271007d27672120b3d0040ebabc6667d646de68734b1702fee` |
| `/home/marian/Downloads/map_projector_info.yaml` | 4,733 | `614af74d6aaefe052c435338b585797a451e7922efc290329374c2abd2b8992d` |
| `/home/marian/Downloads/pointcloud_map_georeferenced.pcd` | 16,069,688 | `6121b6bf8df4324ab42286822f0e6f00c67ecc10daecf84d180129a6069ef9a3` |

The validator checked these hashes before and after execution. It also checked
every published XYZ float against the original binary PCD: exact equality for
all 1,004,344 points, preserving their order.

All repository changes are new files under `src/atlas_map_integration_demo/`:

- `package.xml`, `CMakeLists.txt`: test-only package and dependencies.
- `launch/campus_map.launch.py`: pinned map components, routing probe, RViz.
- `config/campus_map.rviz`: original PCD, Autoware road markers, route, endpoints,
  labels, grid, and coordinate axes, all in `map`.
- `src/map_routing_probe.cpp`: deserializes the actual loader publication,
  constructs the planner-rule routing graph, validates a route, publishes it,
  and exports diagnostic geometry/statistics.
- `scripts/run_validation.py`: bounded runtime/DDS/node/RViz validation.
- `scripts/analyze_alignment.py`: read-only occupancy/elevation diagnostics.
- `scripts/build_demo.sh`: reproducible pinned-source/dependency checks and
  missing-package build selection; never invokes installers.
- `README.md`, this report, and `results/`: saved logs, numerical evidence,
  route CSV, and a diagnostic plot. The plot is not an RViz screenshot.

Pinned source revisions used:

| Repository | Revision |
|---|---|
| Autoware root, tag 1.9.0 | `10718787ba6e28f038a0cb29ff99cc627b5abfd2` |
| `autoware_core`, tag 1.9.0 | `f25f83c632c1984ec276c894c41857d4abc0dad8` |
| `autoware_lanelet2_extension` | `4a3420d8cc19906e7739618f8a1686400f79b4ac` |
| `autoware_universe` reference | `6e477c645efec33f7909095eea684474e97f5e3d` |

The upstream repositories remained clean. Before compilation, rosdep checked the
complete loader/visualizer dependency closure using the existing local database;
all system build/export/runtime dependencies were satisfied. Eighteen cached
packages were reused from the deskew, NDT, and vehicle-simulator overlays.

Only seven missing upstream packages were built:
`autoware_localization_msgs`, `autoware_system_msgs`,
`autoware_component_interface_specs`, `autoware_geography_utils`,
`autoware_map_projection_loader`, `autoware_map_loader`, and
`autoware_lanelet2_map_visualizer`. All finished in approximately 82 seconds,
with sequential package execution and two compiler jobs. The Atlas C++ probe
compiled and linked successfully on its first build, in approximately 19 seconds.
Later launch/script fixes only refreshed the local install; upstream was not rebuilt.

Build directories:

- `/tmp/atlas_map_upstream_build`, `/tmp/atlas_map_upstream_install`,
  `/tmp/atlas_map_upstream_log`
- `/tmp/atlas_map_demo_build`, `/tmp/atlas_map_demo_install`,
  `/tmp/atlas_map_demo_log`
- `/tmp/atlas_map_dependency_audit.log`, `/tmp/atlas_map_closure.tsv`,
  `/tmp/atlas_map_build_selection.json`

## 2. Components and original-map loading

Executed components:

| Package | Plugin |
|---|---|
| `autoware_map_projection_loader` | `autoware::map_projection_loader::MapProjectionLoader` |
| `autoware_map_loader` | `autoware::map_loader::Lanelet2MapLoaderNode` |
| `autoware_map_loader` | `autoware::map_loader::PointCloudMapLoaderNode` |
| `autoware_lanelet2_map_visualizer` | `autoware::lanelet2_map_visualizer::Lanelet2MapVisualizationNode` |

The supplied projector YAML was read unchanged: `LocalCartesian`, `WGS84`, origin
latitude 51.08080935188234, longitude -114.1331143659806, altitude 0.0. The actual
projector message was checked against these values. The pinned loader forces
origin altitude to 0.0. No alternative projector or frame transform was applied.

The original OSM lacks `MetaInfo` version metadata, so
`allow_unsupported_version=true` is necessary for this file. Autoware emitted the
expected missing-format-version warning and published the map successfully.
Centerlines were generated at 1 m resolution with `use_waypoints=false`; the OSM
itself was not edited. PCD whole-map loading was enabled without downsampling;
the pinned single-PCD fallback computed metadata in memory.

The pinned Lanelet2 loader actually publishes on absolute `/map/vector_map`.
The demo explicitly remaps that source name to `/atlas_map_demo/vector_map`.
The projector interface remains the pinned absolute `/map/map_projector_info`.
ROS domain 77 isolates these standard interfaces from other demonstrations.

## 3. Runtime publications and startup times

Successful run: `/tmp/atlas_map_demo_results/run_20261012_022357`.
Evidence is also saved in [results/runtime_validation.json](results/runtime_validation.json),
[results/publications.json](results/publications.json), and
[results/runtime.log](results/runtime.log).

Times below are the independent Python subscriber's first receipt since launch;
they include loading, discovery, serialization, and subscriber processing, and
are not isolated loader benchmarks.

| Topic | Message type | Payload | Frame | First receipt |
|---|---|---|---|---:|
| `/map/map_projector_info` | `autoware_map_msgs/msg/MapProjectorInfo` | supplied origin/projector verified | no header | 0.572 s |
| `/atlas_map_demo/vector_map` | `autoware_map_msgs/msg/LaneletMapBin` | 8,927,167 bytes | `map` | 1.473 s |
| `/atlas_map_demo/pointcloud_map` | `sensor_msgs/msg/PointCloud2` | 1,004,344 points / 16,069,504 bytes | `map` | 1.557 s |
| `/atlas_map_demo/road_markers` | `visualization_msgs/msg/MarkerArray` | 2,742 markers | every marker `map` | 6.662 s |
| `/atlas_map_demo/route` | `autoware_planning_msgs/msg/LaneletRoute` | 10 segments | `map` | 6.665 s |
| `/atlas_map_demo/route_path` | `nav_msgs/msg/Path` | 160 poses | `map` | 6.669 s |
| `/atlas_map_demo/diagnostic_markers` | `visualization_msgs/msg/MarkerArray` | 7 endpoint/label/axis markers | every marker `map` | 6.670 s |

Map/route timestamps were nonzero. The upstream static PCD publication uses
timestamp zero; this is recorded rather than rewritten. All displays use the
same fixed frame, so this static map view needs no map-to-robot transform.

The successful run lasted 16.863 seconds, including an eight-second hold after
all topics arrived. Loader, visualizer, probe, container, and RViz nodes were
present and alive during the hold. Launch and validator exited 0. No critical
loader, routing, or RViz runtime errors were logged.

Initial runtime attempt failed for local integration configuration reasons:

1. Humble flattened `[LaunchConfiguration(...)]` into a scalar PCD parameter;
   the loader rejected it because it requires a string array. The launch now
   preserves an array of substitution sequences.
2. The first launch remapped obsolete relative `output/lanelet2_map`, leaving
   the real publication on `/map/vector_map`; the probe timed out. The remap
   now uses the absolute pinned interface.
3. The first failed run's group SIGINT caused a duplicate shutdown signal and
   RViz exit -2. The validator now signals launch only, letting launch forward
   shutdown once. The successful run exited cleanly.

The failed attempt is preserved at
`/tmp/atlas_map_demo_results/run_20261012_022222`. It is not counted as a passed
milestone. No dataset repair or dependency installation was needed to resolve it.

## 4. Executed routing graph and route

The probe consumed and deserialized the actual `LaneletMapBin` from Autoware.
It did not parse the OSM independently with a different projection. It called
the pinned `instantiate_routing_graph_and_traffic_rules` with
`lanelet::autoware::DefaultLocation` (`autoware`) and `Participants::Vehicle`,
matching the main graph in core `RouteHandler::setMap`.

The custom Autoware rules inherit Germany/Vehicle rules and additionally permit
appropriately tagged areas. This is the selected runtime policy, not a claim
that German defaults represent campus operating rules. A second Germany/Vehicle
graph was executed for comparison; its lanelet vertex, successor-edge, lane-change,
and weak-component counts matched this graph.

| Statistic | Executed result |
|---|---:|
| Lanelets in loaded map | 2,225 |
| Routable lanelet vertices | 2,225 |
| Original lanelets denied by vehicle rules | 0 |
| Inverted routable vertices | 0 |
| Successor edges | 2,547 |
| Lane-change edges | 0 |
| Weakly connected components | 5 |
| Weak-component sizes | 2,170; 28; 14; 10; 3 |
| Strongly connected components | 974 |
| Largest strongly connected component | 1,243 |
| `RoutingGraph::checkValidity(false)` errors | 0 |

Component statistics cover directed lanelet successors/lane-change edges, not
area nodes or conflict/adjacent-but-not-routable edges. Weak components ignore
edge direction; strong components retain it. These distinctions matter: even
inside the largest weak component, arbitrary ordered endpoints are not guaranteed
to have a route. Fifty-five lanelets are outside that largest component. Their
disconnection is functional, even though the graph validity check reports no errors.
Whether each segment is intentionally isolated requires map-author review.
The disconnected three-lanelet component is directly relevant to the research
cloud: lanelets **10697, 22398, and 54548** form the curved loading-dock-region
road, spanning approximately X -75.995..33.139 m and Y -118.571..42.407 m.
It has no lanelet graph connection to the main campus component. A separate
ten-lanelet component also intersects the north side of the cloud footprint.
The validated route deliberately belongs to the requested largest component,
on the nearby main road, rather than claiming a route from the dock. Actual
dock-to-campus routing requires an author-verified connection or confirmation
that isolation is intentional; no connector was invented.

The probe selected a start near the PCD region in the largest weak component,
searched reachable successor-only destinations, and called the actual
`RoutingGraph::getRoute(start, goal, 0, false)`. The resulting shortest path was
verified independently against both `following(..., false)` and
`previous(..., false)` and `traffic_rules->canPass(from, to)` for every transition.
Both endpoints and every route lanelet are in the largest weak component.

| Route measurement | Result |
|---|---|
| Start lanelet | 38184, forward |
| Goal lanelet | 35280, forward |
| Lanelet count | 10 |
| Centerline route length | **146.063561431 m** |
| Maximum centerline join gap | **0.000000 m** |
| Published path poses | 160, including shared join samples |
| Start XYZ | (41.638426, 68.996451, -0.000509) m |
| Goal XYZ | (187.676290, 68.374729, -0.003122) m |
| Graph/route diagnostics and exports | approximately 1.164 s |

Lanelet sequence:
`38184 → 23892 → 55243 → 17419 → 48352 → 17626 → 17829 → 48462 → 6031 → 35280`.

The probe's published route contains those exact preferred primitives; the
independent ROS subscriber checked the sequence. Graph computation, valid
transitions, nonempty path, finite/plausible length, and exact publication were
executed and verified. This is not success inferred from map loading. The
Autoware mission-planner node/service was **not** executed; downstream acceptance
of this route is a separate next-stage test.

Evidence: [results/routing.json](results/routing.json),
[results/route.csv](results/route.csv), and
[results/route_geometry.json](results/route_geometry.json).
Component locations and examples are saved in
[results/component_geography.json](results/component_geography.json).

## 5. Warning causes and functional impact

The earlier installed validator used UTM and default Germany vehicle rules.
Its 81,515 warnings are retained as diagnostics, not reclassified as runtime
loader errors. Its log is `/tmp/atlas_summer_map_lanelet_validation.log`.
Representative primitives were inspected against the original OSM and the
geometry actually loaded by Autoware. Runtime geometry measurements are separate
from the earlier validator's warning counts.

| Category | Earlier count | Representative cause | Actual loading/routing impact |
|---|---:|---|---|
| Curvature | 7,694 on 330 lanelets | Lanelet 55813 has densely sampled curved bounds (ways 55811/55812). Its actual generated centerline still reaches 0.628 m⁻¹. | Does not prevent loading or graph routing. Can make a geometric route physically infeasible. Twenty-six generated centerlines exceed 0.5 m⁻¹; maximum observed is 0.963 m⁻¹ on lanelet 11935. |
| Mandatory tags | 4,168 | 4,150 missing `type`, 9 missing `subtype`, 9 missing `location`. All 3,774 road boundary ways lack `type`; way 55811 is an example. | Loaded successfully and all road lanelets pass vehicle rules. Boundary semantics/lane-change permissions cannot be assumed. The executed graph has zero lane-change edges. Missing markings should be supplied from evidence, not invented. |
| Close points | 13,099 | Nodes 56437 and 42275 are approximately 8.3 mm apart; subsequent nearby pairs are centimetres apart. Boundaries contain 8,393 segments shorter than 1 cm and 14,473 shorter than 10 cm, among 51,116 segments. | Dense export/resampling creates many warnings; there are no zero-length consecutive boundary segments in the loaded dump. It does not block the demonstrated route. Nearby distinct IDs must not be merged blindly because identity defines topology. Near-coincident features and sharp connectors need local review. |
| Unknown tags | 56,554 | 55,669 are the nonstandard `h` key, e.g. node 13726188000; the remainder is ordinary OSM parking, access, building, entrance, etc. metadata. | Mostly systematic export metadata. Loading and route computation succeeded. `h=0` is not recognized elevation evidence and does not fix missing `ele`. Some access tags can be semantically important; do not blanket-delete metadata. |

The raw OSM also repeats 70,422 node records and 4,910 way records identically,
and repeats `h` within 126,034 node records. No conflicting duplicates or dangling
primitive references were found. The loader tolerated this redundancy.

For the selected route, sampled centerline curvature is much smaller:
maximum 0.059486 m⁻¹, 95th percentile 0.016675 m⁻¹. Sampled lane width is
approximately 2.192–2.199 m. These are geometric checks, not surveyed road-width
or vehicle-clearance certification. The near-uniform width is consistent with
systematic map construction and needs provenance from the Lanelet2 author.

No repair was necessary to demonstrate loading and successor routing. No derived
OSM was created. Priorities are disconnected road segments, lane-change semantics,
vehicle-feasible curves, measured widths, and elevation/alignment—not eliminating
all warning counts.

Evidence: [results/warning_analysis.json](results/warning_analysis.json) and
[results/geometry_diagnostics.json](results/geometry_diagnostics.json).

## 6. Quantitative PCD / Lanelet2 alignment assessment

All measurements below retain the original PCD and the loader's actual
LocalCartesian road geometry. No registration transform was estimated or applied.

| Axis | PCD minimum / maximum (m) | Runtime road-boundary minimum / maximum (m) |
|---|---|---|
| X / east | -79.563538 / 122.839691 | -1342.402205 / 1196.371060 |
| Y / north | -76.590050 / 119.276711 | -1600.052188 / 251.687697 |
| Z / up | -5.397558 / 21.731119 | -0.288288 / -0.000043 |

The cloud spans 202.403 × 195.867 m horizontally. Its bounding-box area is
39,644 m², only 0.843% of the full road-map bounding-box area; that ratio is not a
measured road-surface coverage fraction. Twenty-four lanelets have runtime
centerline samples inside the PCD rectangle (1,021 samples, including repeated
spatial coverage by different lanes).

As a coverage/elevation diagnostic, the script forms 1 m XY cloud cells with at
least ten points, records their minimum Z, and compares them with runtime
centerline samples. Only 79 of the 1,021 local samples (7.74%) lie within 1.5 m
of such populated cells. Across all local centerline samples, nearest populated
cell distances have median 9.893 m and 90th percentile 26.336 m. **These are
occupancy distances, not alignment residuals**: unscanned asphalt is not a
matched structural feature.

For the 79 nearby samples, lower-envelope PCD Z minus lanelet Z is:

| Statistic | Difference |
|---|---:|
| Minimum | -2.785 m |
| 10th percentile | -2.411 m |
| Median | -1.836 m |
| 90th percentile | +0.398 m |
| Maximum | +1.065 m |

The lower envelope can contain outliers, vegetation, or structures; it is not a
certified road-surface detector. Its heterogeneity does not support applying a
constant vertical correction. In particular, neither -1.836 m nor the metadata's
GNSS antenna altitude 1094.7826704429463 m was used as a transform.

Candidate correspondence investigation found no labelled entrance/kerb/building
nodes within the cloud footprint and no overlapping tagged building footprint.
There are overlapping parking polygons, including way 29019480 with bounds
(-15.111, -9.562) to (37.656, 28.563) m, and street-side parking way 1323546550.
These are possible field-survey regions, not verified matches to cloud features.
Lane boundaries have no physical marking/curb types. There are therefore no
defensible, independently identified feature pairs from which to report an XY
registration RMSE or yaw/origin correction. The static overlay establishes
regional co-location only.

Distinguishing possible causes:

- **Projection:** actual LocalCartesian support and the supplied origin were
  confirmed at runtime. The YAML describes spherical-ENU cloud georeferencing
  rather than exact WGS84 ellipsoidal ENU; its claimed approximately 0.30 m maximum
  discrepancy cannot be reproduced without the absent script/report. Changing
  to UTM/MGRS would introduce another frame change and is not justified.
- **Origin/yaw:** regional overlap is observed, but exact cloud georeferencing
  origin, yaw, scale, and transform provenance are not embedded in the PCD.
  No origin/yaw error is independently determined.
- **Elevation:** all road-boundary nodes lack `ele`, so their loaded heights are
  effectively zero apart from tangent-plane curvature. The metadata describes
  a cloud frame tied to GNSS antenna height, with ground near -2.05 m. Pinned GNSS
  projection uses origin altitude zero, so an uncorrected approximately 1094.78 m
  antenna altitude would not match this cloud's relative height frame. This
  metadata discrepancy is real, but is not a measured correction to apply.
- **Reconstruction:** no raw scans, trajectory, georeferencing residuals, or
  control points were supplied, so SLAM distortion, angular bias, scale error,
  and local structural mismatch cannot be separated quantitatively.

The validated route has only 81.204 m (55.595%) inside the PCD XY rectangle and
64.859 m outside it. A route inside the rectangle is not automatically supported
by dense localization features either. Do not claim real-cloud NDT readiness
for the complete demonstrated route.

Needed information: the PCD author's exact georeferencing script and transform,
trajectory/raw-scan IDs, antenna/LiDAR extrinsics and height convention; the OSM
author's boundary-generation method, measured widths and elevation source; and
at least three well-distributed independently identified non-collinear XY tie
points, plus surveyed ground elevations or equivalent GNSS/trajectory controls.
Corresponding curb/building corners must be identified in both datasets rather
than inferred from similarly shaped outlines.

Evidence: [results/alignment.json](results/alignment.json) and
[results/runtime_map_route.png](results/runtime_map_route.png).

## 7. RViz validation and frame configuration

The actual desktop RViz process launched, initialized OpenGL 4.5, and remained
alive alongside the loaders and probe. Its DDS subscribers were explicitly
confirmed for original PCD, Autoware road markers, route Path, and diagnostics.
All messages/markers use `map`; RViz's fixed frame is `map`. Start and destination
spheres/labels and XYZ arrows were published. No fictitious TF was introduced:
the fixed frame and data frame are identical, so TF identity is sufficient.

Runtime visualization configuration and subscription validation **passed**.
The rendered desktop image was **not visually inspected**. Known GNOME/Wayland
screenshot restrictions were not retried. The saved Matplotlib plot is an
independent geometry diagnostic, not proof of RViz rendering or a screenshot.

## 8. Driving readiness and minimum next pipeline

The executed route is suitable for the next **trajectory-generation experiment**.
It is not a controller-ready trajectory: `nav_msgs/Path` contains geometry without
a velocity/acceleration schedule, and `LaneletRoute` contains lane primitives.
No path generator, velocity planner, controller, or simulator was run in this task.

The smallest source-supported lane-following pipeline to investigate next is:

```text
validated LaneletRoute + vector map + map-frame odometry
  -> autoware_path_generator                  (PathWithLaneId)
  -> autoware_behavior_velocity_planner       (Path and velocity/stop behavior)
  -> autoware_planning_topic_converter        (PathToTrajectory)
  -> autoware_velocity_smoother               (bounded Trajectory)
  -> autoware_trajectory_follower_node        (Control;
       pure-pursuit or MPC lateral + PID longitudinal)
  -> existing autoware_simple_planning_simulator / IDEAL_STEER_ACC
  -> odometry, acceleration, steering/status feedback
```

Core 1.9.0's planning launch uses this path generator and converter, with
`autoware_motion_velocity_planner` between conversion and final smoothing for
obstacle handling. That obstacle stage and its perception inputs are required
when moving beyond an explicitly obstacle-free kinematic test. The actual
mission planner should later validate route acceptance/state and goal handling;
the current graph probe does not substitute for that node's state machine.

Additional readiness inputs are vehicle parameters, nearest-lane settings,
fresh map-frame odometry/acceleration, steering report, and operation-mode state.
The controller outputs `autoware_control_msgs/msg/Control`, which the existing
simulator accepts on `input/ackermann_control_command`; gear/engage state also
needs explicit test-only handling. A command gate and mode/state management must
be considered for an operational stack, even if a bounded isolated simulation
eventually connects controller output directly.

Blockers to resolve before closing the loop:

1. **Frames/initial pose:** the old synthetic run used `atlas_sim_odom`, not `map`.
   Configure a separate test simulator with its origin frame `map`, initialize
   from the selected route's actual pose/yaw, and use a unique simulated base
   frame. Do not connect origin-zero odometry to this campus route by pretending
   the frames already coincide.
2. **Geometry:** the old 1.20 m wheelbase and dimensions are synthetic. With that
   test wheelbase, the selected route's maximum sampled curvature implies about
   0.0713 rad steering, below the old 0.45 rad limit. This supports a limited
   feasibility experiment; it does not certify measured Atlas geometry. Some
   other map centerlines exceed the synthetic model's roughly 0.403 m⁻¹ curvature
   capability and must not be selected blindly.
3. **Velocity/behavior:** no explicit map speed limits are supplied. Inherited
   German urban defaults are not an approved UGV speed policy. Choose a documented
   test speed/acceleration/jerk cap and verify goal stopping. Static graph routing
   does not implement traffic-light right-of-way or obstacle avoidance.
4. **Route semantics:** zero lane-change edges and disconnected segments limit
   route choice. Prove downstream acceptance of the 10-segment route and path
   generation rather than assuming an arbitrary successful graph path is accepted.
   The loading-dock three-lanelet component is disconnected from the main road,
   so this successful main-component route does not enable departure from the
   dock. Review this connection with the OSM author before selecting a dock route.
5. **Elevation/localization:** unverified cloud registration and incomplete route
   coverage block a claim of real-cloud NDT/GNSS readiness. A kinematic simulator
   using ground-truth map-frame pose can test route following without first
   proving PCD registration; that would be explicitly a ground-truth simulation.

**Exact next implementation step:** audit the dependency closure for the pinned
path generator, behavior velocity planner, topic converter, and velocity smoother;
build only missing packages in a new `/tmp` overlay; run a trajectory-only test
using this actual published route/map and explicitly labelled map-frame feedback
fixtures. Validate finite continuous trajectory points, bounded speed and
acceleration, vehicle-feasible curvature, and a zero-speed destination. Keep the
controller and simulator disconnected until that test passes and frame/geometry
choices are documented. Then a separate bounded ground-truth closed-loop
simulation can be proposed and executed. Real-cloud alignment/localization
validation remains a separate required step before autonomous campus driving.

## 9. Reproduction and saved evidence

See [README.md](README.md) for build and interactive/bounded launch commands.
Runtime defaults retain the original filenames and projector. The original map
is deliberately not normalized just to suppress warnings.

The `results/` directory preserves the successful numerical checks and log.
Large centerline/boundary dumps remain in the successful `/tmp` run directory.
All 15 runtime checks passed; the initial failed launch is documented above.
Closed-loop motion, actual mission-planner execution, calibrated registration,
real-data localization, and visual inspection of the RViz image remain unclaimed.
