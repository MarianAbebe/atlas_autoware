# Provisional Engineering Requirements — Pending Comparison with Official ROAM PDR

This matrix uses the engineering objectives supplied for handoff. It is not a
quotation from, or claim of compliance with, the unavailable official ROAM PDR.
Status is limited to the evidence listed here.

| ID | Capability | Implementation | Evidence | Status | Next Test |
|---|---|---|---|---|---|
| REQ-01 | LiDAR acquisition and preprocessing | Atlas synthetic scene publishes timestamped PointCloud2; `atlas_sensor_kit` converts cloud formats; pinned Autoware distortion correction, ground filtering and clustering are composed in the full stack. | [`scene.py`](../../src/atlas_full_stack_demo/scripts/scene.py), [`pointcloud_conversion.cpp`](../../src/atlas_sensor_kit/src/pointcloud_conversion.cpp), [`REPORT.md`](../real_data/engg_loading_dock_01/PERCEPTION.md): five recorded scans passed pinned ground filtering/clustering. Synthetic motion-compensation assumptions are configured in [`full_stack.launch.py`](../../src/atlas_full_stack_demo/launch/full_stack.launch.py); this is not proof of calibrated physical deskew. | PARTIAL | On native Ubuntu, verify frames, acquisition stamps, rates and stage diagnostics; separately validate deskew with measured motion/IMU inputs. |
| REQ-02 | Localization | Pinned NDT and EKF are connected; bootstrap helper initializes map-frame pose; watchdog monitors freshness. Full-stack NDT uses generated synthetic map geometry, not the real campus PCD. | [`localization_loop.launch.py`](../../src/atlas_localization_closed_loop_demo/launch/localization_loop.launch.py), [`initialize_localization.py`](../../src/atlas_full_stack_demo/scripts/initialize_localization.py), [`VALIDATION.md`](../../src/atlas_localization_closed_loop_demo/VALIDATION.md), synchronized stationary evidence summarized in [`VALIDATION.md`](VALIDATION.md). | PARTIAL | Native test: verify NDT/EKF, map→base TF, frame/stamp continuity and watchdog status. Establish map-consistent localization against the actual PCD only after registration is defensible. |
| REQ-03 | Obstacle detection | Pinned ground filter and Euclidean clustering feed Atlas object adaptation and obstacle-stop planning. | [`full_stack.launch.py`](../../src/atlas_full_stack_demo/launch/full_stack.launch.py); five actual recording scans passed preprocessing/clustering in [`PERCEPTION.md`](../real_data/engg_loading_dock_01/PERCEPTION.md); historical full-stack obstacle validation in [`REPORT.md`](../full_stack/REPORT.md) is intermittent and includes freshness failures. | PARTIAL | Verify object topic publisher, frame, timestamps and cluster/object lineage in a bounded controlled synthetic-obstacle test; require three valid stops without watchdog-induced false passes. |
| REQ-04 | Route planning | Campus Lanelet map loader and route adapter generate the route/path used by the full-stack synthetic scene. | [`campus_map.launch.py`](../../src/atlas_map_integration_demo/launch/campus_map.launch.py), [`route_adapter`](../../src/atlas_closed_loop_demo/src/route_adapter.cpp), [`VALIDATION.md`](../../src/atlas_map_integration_demo/VALIDATION.md), stationary bag evidence. | VERIFIED (simulation route publication only) | On native Ubuntu, verify loaded map/projector hashes, route endpoints, frame and path publication. |
| REQ-05 | Trajectory generation | Atlas scene supplies a short route-derived trajectory; pinned motion-velocity planner and velocity smoother publish obstacle and smoothed trajectories. | Full-stack launch and [`REPORT.md`](../full_stack/REPORT.md); synchronized stationary recording reported raw, obstacle and smoothed trajectory publication for over 103 seconds. Historic trajectory freshness faults are documented in [`REPORT.md`](../regressions/obstacle_watchdog/REPORT.md). | PARTIAL | Check continuity, velocity profile, constraints and planner/smoother freshness in the native run; preserve any interruption as a failure. |
| REQ-06 | Vehicle control | Pinned MPC/PID consumes EKF odometry and trajectory, publishing commands through the vehicle command gate to the simulator. | [`localization_loop.launch.py`](../../src/atlas_localization_closed_loop_demo/launch/localization_loop.launch.py) remaps controller odometry to EKF; stationary evidence confirms commands published, which alone is not closed-loop tracking. Historical localization validation reports two of three nominal attempts passed. | PARTIAL | After explicit separate authorization, run controlled clear-route trials; measure tracking, stopping, estimation error and faults. No engagement is part of the handoff baseline. |
| REQ-07 | Obstacle stopping | Pinned obstacle-stop planner receives cluster-derived predicted objects; gate enforces authorization and safety. | [`object_only_stop.param.yaml`](../../src/atlas_full_stack_demo/config/object_only_stop.param.yaml), [`obstacle_stop.param.yaml`](../../src/atlas_obstacle_stopping_demo/config/obstacle_stop.param.yaml), [`VALIDATION.md`](../../src/atlas_obstacle_stopping_demo/VALIDATION.md); prior runs were intermittent and watchdog stops are not valid planned stops. | PARTIAL | Controlled obstacle scenario: prove object lineage, planner stop insertion, positive clearance and stationary hold; repeat three times and classify watchdog faults as failures. |
| REQ-08 | Safety monitoring | Base localization/feedback watchdog and combined perception guard check freshness and authorization; command gate withholds control unless authorized. | [`localization_watchdog.py`](../../src/atlas_localization_closed_loop_demo/scripts/localization_watchdog.py), [`perception_guard.py`](../../src/atlas_perception_planning_demo/scripts/perception_guard.py), [`VALIDATION.md`](VALIDATION.md), obstacle watchdog regression reports. Healthy stationary period >103 s and intentional fault latches are documented; older graph-wide scheduling faults also correctly latched. | PARTIAL | Native stationary validation then controlled stale/missing/frozen input regressions; preserve 300 ms semantics and verify no automatic rearm. |
| REQ-09 | Full-stack integration | ROS graph connects simulated sensing through preprocessing, localization, perception, planning, control, gate and simulator feedback. | [`full_stack.launch.py`](../../src/atlas_full_stack_demo/launch/full_stack.launch.py), [`REPORT.md`](../full_stack/REPORT.md), synchronized stationary bag. Topic publication/connectivity is evidenced; autonomous driving is not established by stationary operation. | PARTIAL | On native host, capture graph lineage and repeat stationary validation; later perform separately authorized clear-route and obstacle scenarios. |
| REQ-10 | Reproducibility | New preflight, pin verification, dependency-plan, build, package smoke-test and stationary validation tools are supplied; maps use environment-selected directory and checksums. | [`NATIVE_UBUNTU_TESTING.md`](NATIVE_UBUNTU_TESTING.md), [`tools/native_ubuntu`](../../tools/native_ubuntu), backup verification documented in [`REPOSITORY_AUDIT.md`](REPOSITORY_AUDIT.md). No clean x86_64 build has been executed. | IMPLEMENTED / UNVERIFIED | Run dependency setup, clean build and no-graph smoke test on Ubuntu 22.04 x86_64; record exact OS, pins, commands and outcomes. |

## Implemented versus experimentally verified

Implemented: synthetic full-stack launch, pinned Autoware component integration,
safe localization-only initialization, real organized PointCloud2 conversion,
bounded real-scan ground/clustering harness, two freshness watchdogs, lightweight
RViz configuration, and the new portable setup/validation tooling.

Experimentally verified in prior evidence: five recorded LiDAR scans through
pinned ground filtering and clustering; standalone/synthetic localization tests;
stationary full-stack localization, perception, route/trajectory, controller
command publication and watchdog operation for a bounded interval; intentional
safety-fault handling. The current release preparation has not been built on a
clean x86_64 host.

Not verified: reliable engaged driving, repeatable full-stack obstacle stopping,
localization against the actual campus PCD, calibrated real sensor extrinsics or
deskew, physical vehicle interfaces, and official PDR compliance.

## Continuation roadmap

1. **Tomorrow, P0:** Reproduce dependencies, map hashes, clean build and no-graph
   smoke test on native Ubuntu 22.04 x86_64. Evidence: terminal logs, exact pin
   report and clean build result.
2. **P0:** Run three stationary tests with localization, both armed watchdogs,
   fresh inputs, no skips, disengaged simulator and synchronized evidence. The
   supplied validation tool performs one run; invoke it separately three times.
3. **P0, separately authorized:** Controlled clear-route operation, then obstacle
   stop, then localization-loss stop. Never infer those capabilities from the
   stationary test or bypass a safety latch.
4. **P1:** Profile scheduler contention and Atlas callback CPU/deserialization
   costs with synchronized process telemetry before proposing runtime changes.
5. **P2:** Measure physical sensor/vehicle transforms, time synchronization,
   actuator/feedback contracts and actual-map registration. Complete bench and
   closed-course review before hardware motion.
6. Reconcile every provisional ID against the official PDR when it is available;
   replace this provisional baseline only through reviewed evidence and team
   agreement.
