# PRD gap-closure status — 2026-10-31

This development-worktree addendum records the checkpointed implementation and
available demonstration evidence. It supplements, and does not replace, the
PRD v0.2 traceability matrix or the provisional engineering requirements.
`PDR_ALIGNED_REQUIREMENTS.md` remains provisional until compared with the
official ROAM PDR.

## Evidence baseline

Checkpoint: `ed82c0a19a77bd06691269d087bbacadaebe4317` (`chore: checkpoint
Atlas demo and PRD development state`). The checkpoint preserves 38 reviewed
source/configuration/test/documentation changes. The generated
`symlink_install_manifest.txt` was excluded.

The validated live demonstration evidence is retained under
`~/roam/demo_results/`:

- `20261031T001103Z_clear_286895`: engaged synthetic clear segment, route and
  localization displays, visible moving vehicle, goal stop, RViz cleanup.
- `20261031T001412Z_obstacle_287816`: engaged synthetic obstacle stop,
  detections/predictions, zero commanded/gated speed at stop, no safety latch,
  RViz cleanup.
- `20261031T000352Z_clear_284990` and
  `20261031T000733Z_clear_285957` are retained earlier unsuccessful startup
  attempts; they are not counted as successful demonstrations.

These results demonstrate the configured synthetic segment and obstacle-stop
behavior on the ARM64 VM. They do not establish full campus-route completion,
physical vehicle safety, or hardware operation. The stop distance in the
obstacle evidence is center-to-center and is not a vehicle-edge clearance
measurement. `./demo.sh check` passed on the stable checkout without starting a
simulation.

## Requirement status

| PRD capability | Current implementation and evidence | Status | Remaining work / evidence boundary |
|---|---|---|---|
| Pi 5 sensor acquisition | No acquisition service or deployment; synthetic sensors and sensor-kit conversion boundaries exist. | Not implemented | Driver integration, packet/time handling, deployment and physical tests on Pi 5. |
| Jetson Orin processing | ROS packages have ARM64 VM evidence; no Orin build, resource, thermal or deployment evidence. | Not validated | Build and profile on target Orin 16 GB with actual sensor rates. |
| Livox Mid-360S | Livox format adapter and acquisition-time synthetic PointCloud2→CustomMsg path exist; no real driver/sensor test. | Prototype only | Confirm exact hardware model/driver fields, firing metadata, clock and extrinsics on sensor. |
| NovAtel PwrPak7D-E2 | Basic synthetic NavSatFix boundary; no vendor solution/status, PPS, velocity, heading or INS integration. | Not implemented | Vendor driver and full solution/lever-arm/time contract. |
| Five IMX296 cameras | Synthetic synchronized stereo interface exists; no five-camera capture, calibration, real optical model or camera fusion. | Prototype only | Hardware capture, synchronization/calibration, bandwidth and perception integration. |
| LiDAR-inertial localization | Pinned third-party ROS 2 FAST-LIO2 wrapper, tracked synthetic config, Livox adapter, evaluator and pose-only shadow path exist. Host synthetic estimator was previously run; the pose-only forwarding run had 304/304 exact unique pose forwards after analyzer correction. | Isolated synthetic prototype | Reproduce on target host; validate against physical timing/calibration. No operational autonomy fusion. FAST-LIO twist/covariance are not trusted. |
| TF ownership | Existing NDT/EKF full-stack owns map-to-base localization; FAST-LIO emits odom-frame output in a separate launch/domain and pose shadow publishes no TF. | Correctly isolated, integration incomplete | Define a single owner for odom→base_link and map→odom, alignment/reset behavior, and lifecycle before any shared graph. |
| Covariance and estimator fusion | NDT/EKF are operational in the demo; FAST-LIO pose-only output deliberately has no covariance. GTSAM graph absent. | Incomplete | Establish source-supported uncertainty for each sensor/estimator and validate consistency, observability, gating and recovery. No fabricated covariance. |
| GTSAM factor-graph fusion | Requirements/interface boundary documented; no factor graph implementation. | Not implemented | Only after measured sensor models, timing, extrinsics and estimator interfaces are established. |
| LiDAR perception | Autoware filtering/clustering and detected/predicted-object adaptation run in synthetic stack; live obstacle demo produced detections and stop behavior. | Synthetic demo validated; target incomplete | Repeat acceptance campaign with recorded object lineage, clearance measured from vehicle envelope, runtime/fault coverage and representative real data. Camera fusion is absent. |
| Routing and planning | Lanelet2 map route, Autoware trajectory generation/velocity smoothing, obstacle stop. Live demo used the configured short segment. | Synthetic segment demonstrated | Verify full intended route endpoints/map assets, collision checking and repeatability; separate Nav2 work is not operationally connected. |
| Closed-loop vehicle control | Autoware controller, command gate and simulator feedback drove the demonstration vehicle under synthetic localization. | Synthetic demonstration validated | Full route tracking/error metrics and robust fault scenarios; physical skid-steer command/feedback contract remains unimplemented. |
| Wheel encoders / drivetrain | Simulator and vehicle-interface scaffold exist; physical four-wheel motor and encoder feedback are absent. | Not implemented on hardware | Confirm drive protocol, wheel geometry, units, status/command messages, watchdog and bench-test interface. |
| Atlas robot description | Atlas description package is scaffold; demo uses a recognizable visual stand-in attached to existing base transform. | Prototype visual only | Measured Atlas geometry, mass/inertial model, sensor mounting/extrinsics and calibrated TF. |
| Reliability/safety | Watchdogs, perception guard, command gate and failure handling are present; final demonstration runs had no latches, earlier failures retained. | Partial synthetic evidence | Systematic dropout, stale/invalid timestamp, delayed input, component crash/recovery and repeated-stop test matrix. No vehicle safety case. |
| Reproducibility | Root `demo.sh`, handoff guide, focused overlay and `check` command exist; both scenarios were run and evidence retained. | Demo procedure validated on this VM | Clean build/replay on x86_64 Ubuntu 22.04 and pinned target environment; preserve logs and exact overlay provenance. |

## Intended architecture from repository requirements

The project has two architecture levels that must not be conflated:

1. **Prototype/operational demonstration graph:** Autoware supplies map-based
   NDT/EKF localization, LiDAR perception, Lanelet2 routing, trajectory
   generation, control and safety gating. This is the current full-stack
   simulator and demo path. Its localization and TF behavior are the accepted
   operational baseline for that graph.
2. **PRD target graph:** Raspberry Pi 5 acquisition for the sensor suite,
   Jetson Orin processing, FAST-LIO2 or LIO-SAM LiDAR-inertial odometry,
   stereo VINS-Fusion, NovAtel GNSS/INS, and a custom GTSAM factor-graph fusion
   layer, with the PRD's Nav2 planning and four-wheel skid-steer/encoder
   interface. The traceability document treats these as target requirements,
   not as satisfied by the Autoware demo.

### Mandatory, alternative, and experimental components

- **Mandatory PRD target components (per current traceability):** Pi 5 sensor
  acquisition; Jetson Orin processing; Mid-360S; PwrPak7D-E2; five IMX296
  cameras; a LiDAR-inertial estimator; stereo VIO; custom GTSAM fusion; camera /
  LiDAR perception; Nav2 planning; four-wheel skid-steer control with encoder
  feedback. The exact vehicle/interface specifications still require official
  PDR confirmation.
- **Estimator alternative:** The PRD wording “FAST-LIO2 or LIO-SAM” makes those
  two LiDAR-inertial implementations alternatives for the LIO role, not a
  requirement to run both. The current experiment uses a pinned third-party
  ROS 2 FAST-LIO2 port; canonical FAST-LIO is ROS 1 and is not that port.
- **Operational baseline, not target substitute:** Autoware NDT/EKF, routing,
  perception, planning and control are what the tested demonstration runs.
  Their presence does not satisfy PRD Nav2 or the specified fusion architecture.
- **Experimental / not yet integrated:** FAST-LIO2 standalone synthetic and
  pose-shadow path; synthetic sensor streams; standalone Nav2 regression
  fixture; any proposed VINS/GTSAM combination. FAST-LIO2 pose must not enter
  the operational EKF until uncertainty, frames, time, TF ownership and failure
  handling are designed and tested. GTSAM is a target requirement in the
  provisional traceability, but no factor graph has been implemented.
- **Official-document dependency:** `PDR_ALIGNED_REQUIREMENTS.md` explicitly
  says it is provisional. Confirm whether Nav2, exact sensor models, estimator
  choices and deployment split are mandatory against the official ROAM PDR
  before freezing architecture or claiming compliance.

## Recommended next milestone

**PRD interface and localization-fusion design gate (no fusion code yet).**
Produce a reviewed, requirement-traceable interface decision covering: the
official PDR wording; one authoritative TF owner per edge; FAST-LIO odom-frame
pose and timestamp semantics; a defensible pose uncertainty source; NDT/EKF
input capabilities; map↔odom alignment and reset semantics; sensor extrinsics
and clock synchronization; dropout/degraded-mode behavior; and shadow-mode
acceptance criteria. Keep FAST-LIO isolated while this gate is unresolved.

Then independently reproduce the demo overlay on clean x86_64 Ubuntu 22.04 and
run a bounded set of repeat clear/obstacle validations. Do not treat the ARM64
VM demo as target-platform evidence. Physical sensor calibration, acquisition,
vehicle interface and safety validation remain hardware-dependent and are not
covered by the synthetic demonstration.
