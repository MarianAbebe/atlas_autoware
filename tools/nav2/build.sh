#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

set +u
source /opt/ros/humble/setup.bash
set -u
for package in nav2_map_server nav2_planner nav2_controller nav2_behaviors nav2_bt_navigator nav2_lifecycle_manager; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || {
    echo "Missing $package; install ros-humble-navigation2 and ros-humble-nav2-bringup." >&2
    exit 2
  }
done
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
python3 tools/nav2/colcon_sync_io.py --log-base .nav2_cache/log build \
  --base-paths src/atlas_nav2_demo \
  --build-base .nav2_cache/build \
  --install-base .nav2_cache/install \
  --executor sequential \
  --event-handlers console_direct+
