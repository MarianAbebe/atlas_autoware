#!/usr/bin/env bash
set -eo pipefail
atlas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$atlas_root"
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
source .closed_loop_cache/demo_install/setup.bash
source .obstacle_cache/install/setup.bash
source .obstacle_cache/demo_install/setup.bash
if [[ -f .perception_cache/install/setup.bash ]]; then source .perception_cache/install/setup.bash; fi
export ENABLE_AGNOCAST=0 CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
python3 src/atlas_perception_planning_demo/scripts/audit_dependencies.py --build
source .perception_cache/install/setup.bash
python3 src/atlas_closed_loop_demo/scripts/colcon_sync_io.py --log-base .perception_cache/atlas_log build \
 --base-paths src/atlas_sensor_kit src/atlas_distortion_corrector_test \
 --build-base .perception_cache/atlas_build --install-base .perception_cache/atlas_install \
 --executor sequential --packages-skip-build-finished --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source .perception_cache/atlas_install/setup.bash
python3 src/atlas_closed_loop_demo/scripts/colcon_sync_io.py --log-base .perception_cache/demo_log build \
 --base-paths src/atlas_perception_planning_demo --build-base .perception_cache/demo_build \
 --install-base .perception_cache/demo_install --executor sequential --cmake-args -DBUILD_TESTING=OFF
