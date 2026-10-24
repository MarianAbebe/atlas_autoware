#!/usr/bin/env bash
set -eo pipefail
atlas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$atlas_root"
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
source .closed_loop_cache/demo_install/setup.bash
source .obstacle_cache/install/setup.bash
source .obstacle_cache/demo_install/setup.bash
export ROS_DOMAIN_ID=81 RCUTILS_COLORIZED_OUTPUT=0
source .perception_cache/install/setup.bash
source .perception_cache/atlas_install/setup.bash
source .perception_cache/demo_install/setup.bash
python3 src/atlas_perception_planning_demo/scripts/run_validation.py "$@"
