#!/usr/bin/env bash
set -eo pipefail
atlas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$atlas_root"
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
source .closed_loop_cache/demo_install/setup.bash
if [[ -f .obstacle_cache/install/setup.bash ]]; then source .obstacle_cache/install/setup.bash; fi
export ENABLE_AGNOCAST=0 CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
python3 src/atlas_obstacle_stopping_demo/scripts/audit_dependencies.py --build
if [[ -f .obstacle_cache/install/setup.bash ]]; then source .obstacle_cache/install/setup.bash; fi
mkdir -p .obstacle_cache
printf '*\n' > .obstacle_cache/.gitignore
python3 src/atlas_closed_loop_demo/scripts/colcon_sync_io.py --log-base .obstacle_cache/demo_log build \
  --base-paths src/atlas_obstacle_stopping_demo --build-base .obstacle_cache/demo_build \
  --install-base .obstacle_cache/demo_install --executor sequential \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF
