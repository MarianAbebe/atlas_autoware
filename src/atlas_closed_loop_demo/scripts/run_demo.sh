#!/usr/bin/env bash
set -eo pipefail
cd /home/marian/roam/atlas_autoware
source /opt/ros/humble/setup.bash
source /home/marian/roam/atlas_autoware/.closed_loop_cache/install/setup.bash
source /home/marian/roam/atlas_autoware/.closed_loop_cache/demo_install/setup.bash
export ROS_DOMAIN_ID=79
python3 src/atlas_closed_loop_demo/scripts/runtime_preflight.py "$@"
python3 src/atlas_closed_loop_demo/scripts/run_validation.py "$@"
