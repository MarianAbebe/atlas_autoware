# PRD v0.2 traceability matrix

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
| Four-wheel-drive skid-steer control with encoder feedback | `atlas_vehicle_interface` and simulation demonstrate command/mixing concepts, but no physical four-wheel motor interface or encoder feedback loop is validated. | PARTIAL PROTOTYPE | Implement the hardware interface, encoder feedback, fault handling, and controlled vehicle tests. |

## Prototype integration evidence outside target compliance

The current repository supplies a useful single-host synthetic integration
baseline: Lanelet2 map loading, NDT/EKF localization, LiDAR preprocessing and
clustering, Autoware planning/control, watchdogs, and disengaged stationary
validation. Native-Ubuntu preparation tools exist under `tools/native_ubuntu`,
but a clean Ubuntu 22.04 x86_64 build and stationary run remain unverified.

Consequently, the current stack is neither full PRD v0.2 compliance nor evidence
of autonomous-driving or physical-hardware readiness. This handoff preserves and
tests the prototype; it does not replace it with the target architecture.
