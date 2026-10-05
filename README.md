# Atlas ROS 2 workspace

Atlas is an autonomous UGV developed by the ROAM student team. This repository
contains Atlas-specific integration, configuration, descriptions, interfaces,
and tooling for Ubuntu 22.04 and ROS 2 Humble.

Atlas packages remain scaffolds with workspace build/setup tooling. No robot
functionality, sensor launch files, or numerical robot configuration is defined.
Autoware will remain an upstream dependency; it is not forked, copied, cloned,
or vendored here.

## Packages

| Package | Responsibility |
| --- | --- |
| `atlas_description` | URDF/Xacro, robot and sensor frames, meshes, TF/extrinsic structure |
| `atlas_sensor_kit` | Sensor configuration, launch integration, topic remapping |
| `atlas_vehicle` | Vehicle dimensions, kinematics, Autoware vehicle configuration |
| `atlas_vehicle_interface` | Autoware command integration and drivetrain status feedback |
| `atlas_launch` | Top-level Atlas and Autoware subsystem launch integration |
| `atlas_tools` | Development, rosbag, diagnostics, localization initialization, setup tools |

Each Atlas package uses `ament_cmake`. `atlas_tools` also contains source-checkout
workspace preparation and build scripts. Robot implementation files and
dependencies will be added when their requirements are confirmed.

## External Dependencies

Third-party ROS source dependencies are managed with vcstool through
[`atlas_dependencies.repos`](atlas_dependencies.repos). The manifest currently
contains the official Livox ROS 2 driver, `livox_ros_driver2`, and the camera
driver `camera_aravis2`.

The Livox driver is pinned to tag `1.2.6`. It requires Livox-SDK2 tag `v1.3.1`,
built and installed separately as a system dependency before building Atlas.
This combination has been build-validated on Ubuntu 22.04 + ROS 2 Humble + ARM64;
physical Mid-360S validation remains pending hardware testing. Livox-SDK2 is not
a ROS workspace package and is not included in `atlas_dependencies.repos`.

`camera_aravis2` is pinned to tag `1.2.0` and is the selected ROS acquisition
driver for the Atlas VEN USB3 Vision / GenICam cameras. This version has been
build-validated on Ubuntu 22.04, ARM64, and ROS 2 Humble. Compatibility with the
physical VEN-161-61U3C-M01 cameras still requires hardware validation. Resolve
its system dependencies through rosdep rather than arbitrary manual apt commands.

Imports go under `src/external/`, which is ignored by Git. Third-party source
must not be committed or vendored into this repository. Atlas tooling does not
clone dependencies or install software.

## Prepare and build the workspace

Use Ubuntu 22.04 with ROS 2 Humble, colcon, vcstool, and initialized rosdep
already available. Install Livox-SDK2 `v1.3.1` separately following its upstream
instructions, outside this repository. Its shared library
`liblivox_lidar_sdk_shared.so` must be available in a standard system library
directory (the upstream default is `/usr/local/lib`). The build script checks
for the library's presence; it does not verify its version or SDK headers.

From the workspace root in a fresh shell:

```bash
source /opt/ros/humble/setup.bash
vcs import src < atlas_dependencies.repos
bash src/atlas_tools/scripts/prepare_workspace.sh
rosdep install --from-paths src --ignore-src --rosdistro humble -y
bash src/atlas_tools/scripts/build_workspace.sh
source install/setup.bash
```

Preparation must precede rosdep: `livox_ros_driver2` ships its ROS 2 manifest as
`package_ROS2.xml`. Atlas copies it to `package.xml` only when their contents
differ. Missing dependencies or manifests cause a clear failure.

The build script can be invoked from any working directory using its path. It
sources `/opt/ros/humble/setup.bash`, runs preparation again safely, checks for
the Livox SDK shared library, and runs the validated full-workspace command:

```bash
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

These CMake arguments select Livox's ROS 2 and Humble behavior. The script
preserves colcon output and exit status. Resolve ROS/system package dependencies,
including camera dependencies, through rosdep; the separate Livox SDK installation
is still required. No Autoware checkout is needed for this build.

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
