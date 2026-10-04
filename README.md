# Atlas ROS 2 workspace

Atlas is an autonomous UGV developed by the ROAM student team. This repository
contains Atlas-specific integration, configuration, descriptions, interfaces,
and tooling for Ubuntu 22.04 and ROS 2 Humble.

This is a scaffold only. It contains no robot functionality, drivers, launch
files, or numerical robot configuration. Autoware will remain an upstream
dependency; it is not forked, copied, cloned, or vendored here.

## Packages

| Package | Responsibility |
| --- | --- |
| `atlas_description` | URDF/Xacro, robot and sensor frames, meshes, TF/extrinsic structure |
| `atlas_sensor_kit` | Sensor configuration, launch integration, topic remapping |
| `atlas_vehicle` | Vehicle dimensions, kinematics, Autoware vehicle configuration |
| `atlas_vehicle_interface` | Autoware command integration and drivetrain status feedback |
| `atlas_launch` | Top-level Atlas and Autoware subsystem launch integration |
| `atlas_tools` | Development, rosbag, diagnostics, localization initialization, setup tools |

Each package uses `ament_cmake` and currently contains only its manifest, build
file, and responsibility README. Implementation-specific directories and
dependencies will be added when their requirements are confirmed.

## Build the scaffold

With ROS 2 Humble and colcon already installed:

```bash
source /opt/ros/humble/setup.bash
colcon list --base-paths src
colcon build --base-paths src
source install/setup.bash
```

The build only registers these empty packages and installs their READMEs. It
provides no runnable nodes or launch files and requires no Autoware checkout.

## TODO before implementation

- Confirm robot geometry, vehicle dimensions, and kinematic parameters.
- Confirm sensors, mounting locations, frame conventions, and calibrated extrinsics.
- Confirm topic names, message contracts, and hardware/drivetrain interfaces.
- Select the upstream Autoware version and integration requirements.
- Set actual maintainer contacts and choose a repository license. Package manifests
  explicitly use `TODO` metadata and `todo@example.com` as a contact placeholder.

Unknown robot-specific information must remain TODO until verified. Do not use
invented dimensions, transforms, calibration, topic names, or hardware interfaces.
See [docs](docs/README.md) and the package READMEs for future scope.
