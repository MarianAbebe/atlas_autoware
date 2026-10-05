# atlas_tools

Atlas-specific development utilities.

Workspace tooling is available; no robot functionality is implemented.

## Workspace scripts

- `scripts/prepare_workspace.sh` prepares the imported Livox ROS 2 manifest
  idempotently, before rosdep or colcon discovers packages.
- `scripts/build_workspace.sh` sources ROS 2 Humble, prepares Livox, checks for
  the system SDK shared library, and builds with the required Humble CMake flags.

These scripts are run from the source checkout and locate the workspace from
their own paths. They do not import dependencies or install software. See the
root README for onboarding and the separate Livox-SDK2 requirement.

## Intended responsibilities

- Development and setup utilities.
- Rosbag, diagnostics, and localization initialization tools.

## TODO

- Confirmed workflows, interfaces, and topic names needed by each utility.

Add implementation files and dependencies only after these details are confirmed.
