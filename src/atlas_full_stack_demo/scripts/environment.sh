# Source from any directory; existing built dependencies only.
atlas_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$atlas_repo_root"
export ATLAS_REPO_ROOT="$atlas_repo_root"
export ATLAS_AUTOWARE_ROOT="${ATLAS_AUTOWARE_ROOT:-$(cd "$atlas_repo_root/.." && pwd)/references/autoware}"
export ATLAS_MAP_DIR="${ATLAS_MAP_DIR:-$HOME/Downloads}"
source /opt/ros/humble/setup.bash
for overlay in \
  .closed_loop_cache/install .closed_loop_cache/demo_install \
  .obstacle_cache/install .obstacle_cache/demo_install \
  .perception_cache/install .perception_cache/atlas_install .perception_cache/demo_install; do
  if [[ -f "$overlay/setup.bash" ]]; then source "$overlay/setup.bash"; fi
done
if [[ -d "$PWD/.full_stack_cache/cluster_install/share" ]]; then
  export AMENT_PREFIX_PATH="$PWD/.full_stack_cache/cluster_install:${AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$PWD/.full_stack_cache/cluster_install/lib:${LD_LIBRARY_PATH:-}"
fi
if [[ -f .localization_cache/install/setup.bash ]]; then source .localization_cache/install/setup.bash; fi
if [[ -f .localization_cache/demo_install/setup.bash ]]; then source .localization_cache/demo_install/setup.bash; fi
if [[ -f .full_stack_cache/install/setup.bash ]]; then source .full_stack_cache/install/setup.bash; fi
export ROS_DOMAIN_ID=86 PYTHONDONTWRITEBYTECODE=1
