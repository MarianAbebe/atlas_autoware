# PRD v0.2 traceability matrix

## Sprint status — 2026-11-07

The official PDR remains unavailable; these IDs are local traceability IDs, not
quoted PDR identifiers. Status requires the cited evidence rather than merely a
package or launch file.

| ID | Requirement | Existing implementation and evidence | Status | Missing functionality / hardware dependency | Proposed acceptance test |
|---|---|---|---|---|---|
| PRD-01 | Pi 5 sensor acquisition and synchronization | Topic/frame/timestamp/QoS contract in `config/sensor_contracts.yaml`; deterministic fixture in `atlas_sensor_sim`. | IMPLEMENTED_NOT_VALIDATED | Pi service, drivers, PTP/PPS/network timing, measured calibration. | Pi-to-Jetson timestamp/health test with real devices. |
| PRD-02 | Jetson AGX Orin autonomy processing | ARM64 VM source portability only. | BLOCKED_BY_HARDWARE | Orin deployment, resource/thermal test. | Reproducible Orin build and sustained graph telemetry. |
| PRD-03 | LiDAR/IMU odometry | NDT/EKF prototype; FAST-LIO2 Livox adapter plus health/evaluation boundary in `atlas_lio_integration`. | PARTIAL | Installed estimator, measured extrinsics, physical data accuracy evaluation. | Replay calibrated bag with source/receipt health and trajectory scoring. |
| PRD-04 | VINS-Fusion | Stereo/IMU contract and deterministic fixture. | IMPLEMENTED_NOT_VALIDATED | VINS runtime, calibration and synchronized camera data. | Calibrated stereo/IMU replay with frame and monotonic-stamp checks. |
| PRD-05 | NovAtel GNSS/INS | `NavSatFix` contract only; invalid-fix behavior documented in `docs/state_estimation/INTEGRATION_BOUNDARIES.md`. | BLOCKED_BY_HARDWARE | PwrPak driver, PPS/time, solution status, lever arm and field data. | Receiver replay/bench test rejecting invalid fixes. |
| PRD-06 | GTSAM fusion | Factor-graph design boundary documented; no graph is represented as implemented. | NOT_IMPLEMENTED | Factor implementation and estimator health. | Deterministic factor/outlier/reset unit suite, then replay. |
| PRD-07 | Nav2 Ackermann navigation | Historical isolated skid-steer Nav2 fixture passed headless scenarios but is not Atlas drivetrain evidence. `nav2_ackermann.launch.py` uses Smac Hybrid-A*, RPP, bicycle dynamics, ray-cast `/atlas_ackermann/scan`, and a Nav2 obstacle layer. Straight goal runtime passed (`ackermann/open_verified_20261107T070300Z`); blocked route failed safely with zero output (`ackermann/blocked_20261107T072300Z`). | IMPLEMENTED_NOT_VALIDATED | Curved/radius, successful avoidable-obstacle, and complete runtime safety cases plus real estimator/vehicle integration remain unvalidated. QEMU deadline misses also affect multi-second navigation reliability. | Run bounded curved, radius, avoidable, and safety cases on a capable host using the obstacle-aware Ackermann fixture. |
| PRD-08 | Ackermann command and encoder interface | `ackermann_adapter` publishes standard Autoware `Control` (velocity, steering angle/rate, optional acceleration); isolated simulator publishes encoder-derived motion. All geometry is provisional. | PARTIAL | Measured geometry, physical steering/motor/encoder protocol, slip and hardware fault validation. | Bench command/encoder fault test and closed-course review. |
| PRD-09 | Camera/LiDAR perception | LiDAR preprocessing/clustering/object adapter; camera contract only. | PARTIAL | Camera inference, calibration, fusion, hardware data. | Timestamp/frame-preserving multimodal replay. |
| PRD-10 | Safety/fail-safe stopping | Existing Autoware watchdogs retained; isolated Nav2 gate now requires fresh command and localization source/receipt timing. | VERIFIED | Hardware E-stop and actuator verification. | Intentional stale/missing/frozen tests; hardware bench thereafter. |
| PRD-11 | Sustained autonomous laps | One-lap baseline and retained fault evidence; two-lap run stopped on freshness after VM-wide interruption. | DEFERRED_PERFORMANCE | Stable target hardware and scheduler attribution. | Repeated full-stack trials on provisioned hardware. |

This matrix distinguishes the committed Autoware-based prototype from the Atlas
PRD v0.2 target. `PARTIAL PROTOTYPE` means only a reusable interface or synthetic
demonstration exists; it does not mean the target subsystem is integrated.

| Target requirement | Current repository evidence | Status against PRD v0.2 | Remaining target work |
|---|---|---|---|
| Raspberry Pi 5 sensor acquisition | `atlas_sensor_kit` contains Jetson-side point-cloud format adapters; the full-stack handoff generates synthetic inputs on one host. No Raspberry Pi acquisition service or deployed Pi pipeline is present. | NOT IMPLEMENTED | Implement, timestamp, deploy, and hardware-test Pi acquisition. |
| Jetson Orin processing | ROS 2 packages are source-portable and have run in an ARM64 VM, but there is no Jetson Orin deployment or performance evidence. | NOT IMPLEMENTED | Build and validate the target graph on the selected Orin, including resource and thermal limits. |
| Livox Mid-360S LiDAR | Livox-format conversion and synthetic/recorded-cloud tests exist in `atlas_sensor_kit`; the handoff graph does not run a validated Mid-360S driver or physical sensor. | PARTIAL PROTOTYPE | Integrate the pinned driver/SDK, calibration, time synchronization, and physical Mid-360S tests. |
| NovAtel PwrPak7D-E2 GNSS/INS | The prototype uses synthetic odometry/IMU-like inputs and Autoware EKF. No PwrPak7D-E2 driver, messages, or GNSS/INS fusion factors are integrated. | NOT IMPLEMENTED | Integrate and hardware-test the NovAtel input and its timing/frame contracts. |
| Five IMX296 cameras | No five-camera acquisition, calibration, synchronization, or deployed camera graph is present. | NOT IMPLEMENTED | Integrate all five cameras and validate calibration, timestamps, bandwidth, and health reporting. |
| FAST-LIO2 or LIO-SAM | Localization uses pinned Autoware NDT scan matching plus EKF in `atlas_localization_closed_loop_demo`. | NOT IMPLEMENTED | Integrate and validate the selected LiDAR-inertial odometry implementation. |
| Stereo VINS-Fusion | No stereo VINS-Fusion estimator is present. | NOT IMPLEMENTED | Integrate synchronized stereo/IMU inputs and validate VIO on target hardware. |
| Custom GTSAM factor-graph fusion | No custom GTSAM graph is present; the prototype uses Autoware EKF. | NOT IMPLEMENTED | Implement the specified factors, estimator health handling, and independent validation. |
| Camera/LiDAR perception | Pinned Autoware ground filtering and Euclidean LiDAR clustering are exercised with synthetic and recorded clouds. Camera detection and camera/LiDAR fusion are absent. | PARTIAL PROTOTYPE | Add camera inference and calibrated multimodal fusion, then validate with representative data. |
| Nav2 planning | Planning is an Autoware Lanelet2/path-generation/motion-velocity pipeline, not Nav2. | NOT IMPLEMENTED | Design and integrate the PRD Nav2 planning stack without treating Autoware evidence as Nav2 evidence. |
| Ackermann steering control with encoder feedback | `atlas_nav2_demo` now has a simulation-only bounded Ackermann command/encoder interface; the older four-wheel skid-steer mixer is historical fixture evidence only. No physical steering/traction interface or encoder feedback loop is validated. | PARTIAL PROTOTYPE | Measure vehicle geometry, implement the physical traction/steering and encoder interfaces, fault handling, and controlled vehicle tests. |

## Prototype integration evidence outside target compliance

The current repository supplies a useful single-host synthetic integration
baseline: Lanelet2 map loading, NDT/EKF localization, LiDAR preprocessing and
clustering, Autoware planning/control, watchdogs, and disengaged stationary
validation. Native-Ubuntu preparation tools exist under `tools/native_ubuntu`,
but a clean Ubuntu 22.04 x86_64 build and stationary run remain unverified.

Consequently, the current stack is neither full PRD v0.2 compliance nor evidence
of autonomous-driving or physical-hardware readiness. This handoff preserves and
tests the prototype; it does not replace it with the target architecture.
