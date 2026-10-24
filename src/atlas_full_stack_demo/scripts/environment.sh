# Source from any directory; existing built dependencies only.
atlas_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$atlas_repo_root"
export ATLAS_REPO_ROOT="$atlas_repo_root"
export ATLAS_AUTOWARE_ROOT="${ATLAS_AUTOWARE_ROOT:-$(cd "$atlas_repo_root/.." && pwd)/references/autoware}"
export ATLAS_MAP_DIR="${ATLAS_MAP_DIR:-$HOME/atlas_assets}"

# ROS/colcon setup files may read unset variables. Temporarily disable nounset,
# then restore the caller's original setting after every sourced setup file.
_atlas_source_setup() {
  local _atlas_setup_path="$1" _atlas_setup_status _atlas_had_nounset=0
  case $- in *u*) _atlas_had_nounset=1 ;; esac
  set +u
  if source "$_atlas_setup_path"; then
    _atlas_setup_status=0
  else
    _atlas_setup_status=$?
  fi
  if (( _atlas_had_nounset )); then set -u; else set +u; fi
  return "$_atlas_setup_status"
}

_atlas_source_setup /opt/ros/humble/setup.bash
for overlay in \
  .closed_loop_cache/install .closed_loop_cache/demo_install \
  .obstacle_cache/install .obstacle_cache/demo_install \
  .perception_cache/install .perception_cache/atlas_install .perception_cache/demo_install; do
  if [[ -f "$overlay/setup.bash" ]]; then _atlas_source_setup "$overlay/setup.bash"; fi
done
if [[ -d "$PWD/.full_stack_cache/cluster_install/share" ]]; then
  export AMENT_PREFIX_PATH="$PWD/.full_stack_cache/cluster_install${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$PWD/.full_stack_cache/cluster_install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
if [[ -f .localization_cache/install/setup.bash ]]; then _atlas_source_setup .localization_cache/install/setup.bash; fi
if [[ -f .localization_cache/demo_install/setup.bash ]]; then _atlas_source_setup .localization_cache/demo_install/setup.bash; fi
if [[ -f .full_stack_cache/install/setup.bash ]]; then _atlas_source_setup .full_stack_cache/install/setup.bash; fi
unset -f _atlas_source_setup
export ROS_DOMAIN_ID=86 PYTHONDONTWRITEBYTECODE=1
