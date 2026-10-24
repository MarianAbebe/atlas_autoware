#!/usr/bin/env bash
# Interactive development launch only; not a numerical validation verdict.
set -eo pipefail
atlas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$atlas_root"
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
source .closed_loop_cache/demo_install/setup.bash
if [[ ! -f .obstacle_cache/demo_install/setup.bash ]]; then
  echo 'Obstacle demo is not built. Run scripts/build_demo.sh after resolving audited dependencies.' >&2
  exit 1
fi
if [[ -f .obstacle_cache/install/setup.bash ]]; then source .obstacle_cache/install/setup.bash; fi
source .obstacle_cache/demo_install/setup.bash
export ROS_DOMAIN_ID=80
python3 src/atlas_closed_loop_demo/scripts/dds_probe.py --results /tmp/atlas_obstacle_dds_probe
ros2 launch atlas_obstacle_stopping_demo obstacle_stopping.launch.py "$@"
