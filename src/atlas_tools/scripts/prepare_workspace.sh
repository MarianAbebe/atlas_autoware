#!/usr/bin/env bash
set -euo pipefail

script_dir=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
workspace_root=$(cd -- "$script_dir/../../.." && pwd -P)
livox_dir="$workspace_root/src/external/livox_ros_driver2"

if [[ ! -f "$workspace_root/atlas_dependencies.repos" ]]; then
  echo "Error: Cannot locate the Atlas workspace from $script_dir." >&2
  exit 1
fi
if [[ ! -d "$livox_dir" ]]; then
  echo "Error: Missing $livox_dir. Import atlas_dependencies.repos with vcs first." >&2
  exit 1
fi
if [[ ! -f "$livox_dir/package_ROS2.xml" ]]; then
  echo "Error: Missing $livox_dir/package_ROS2.xml. Check the Livox dependency checkout." >&2
  exit 1
fi

# Livox ships separate ROS manifests; colcon and rosdep need the ROS 2 one.
# Avoid rewriting an already prepared manifest on subsequent runs.
if ! cmp -s -- "$livox_dir/package_ROS2.xml" "$livox_dir/package.xml"; then
  cp -- "$livox_dir/package_ROS2.xml" "$livox_dir/package.xml"
fi
echo "Livox ROS 2 manifest ready: $livox_dir/package.xml"
