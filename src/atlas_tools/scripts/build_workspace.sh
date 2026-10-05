#!/usr/bin/env bash
set -eo pipefail

script_dir=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
workspace_root=$(cd -- "$script_dir/../../.." && pwd -P)

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Error: ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash." >&2
  exit 1
fi
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
  echo "Error: ROS_DISTRO=$ROS_DISTRO. Use a fresh shell with ROS 2 Humble." >&2
  exit 1
fi
# ROS setup scripts may reference unset variables, so do not enable nounset.
source /opt/ros/humble/setup.bash
bash "$script_dir/prepare_workspace.sh"

# ROS 2 Livox links the shared SDK, not the ROS 1 static library.
# Check the upstream install location and standard system library directories.
sdk_found=false
for library in /usr/local/lib/liblivox_lidar_sdk_shared.so \
               /usr/lib/liblivox_lidar_sdk_shared.so \
               /usr/lib/*/liblivox_lidar_sdk_shared.so \
               /lib/liblivox_lidar_sdk_shared.so \
               /lib/*/liblivox_lidar_sdk_shared.so; do
  if [[ -f "$library" && -r "$library" ]]; then
    sdk_found=true
    break
  fi
done
if [[ "$sdk_found" != true ]]; then
  echo "Error: Required liblivox_lidar_sdk_shared.so was not found in standard system library directories." >&2
  echo "Install Livox-SDK2 v1.3.1 separately before building Atlas; this script does not install it." >&2
  exit 1
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "Error: colcon is unavailable. Set up the ROS 2 build tools first." >&2
  exit 1
fi

cd -- "$workspace_root"
# These flags select Livox's ROS 2 and Humble-specific CMake paths.
exec colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
