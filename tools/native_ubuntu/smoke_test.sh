#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"
export ATLAS_REPO_ROOT="$repo_root"
export ATLAS_AUTOWARE_ROOT="${ATLAS_AUTOWARE_ROOT:-$(cd "$repo_root/.." && pwd)/references/autoware}"
export ATLAS_MAP_DIR="${ATLAS_MAP_DIR:-$HOME/Downloads}"

for file in finalMap_repaired.osm map_projector_info.yaml pointcloud_map_georeferenced.pcd; do
  [[ -s "$ATLAS_MAP_DIR/$file" ]] || { echo "Missing map asset: $ATLAS_MAP_DIR/$file" >&2; exit 2; }
done
(cd "$ATLAS_MAP_DIR" && sha256sum -c "$repo_root/tools/native_ubuntu/map_assets.sha256")
source /opt/ros/humble/setup.bash
source src/atlas_full_stack_demo/scripts/environment.sh

for pkg in atlas_full_stack_demo atlas_localization_closed_loop_demo atlas_obstacle_stopping_demo \
  atlas_perception_planning_demo atlas_closed_loop_demo atlas_sensor_kit \
  atlas_distortion_corrector_test atlas_lidar_perception_test atlas_map_integration_demo; do
  prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null)" || { echo "Package not discoverable: $pkg" >&2; exit 3; }
  printf 'PACKAGE %s %s\n' "$pkg" "$prefix"
done

full_prefix="$(ros2 pkg prefix atlas_full_stack_demo)"
for path in launch/full_stack.launch.py config/full_stack_lightweight.rviz; do
  [[ -r "$full_prefix/share/atlas_full_stack_demo/$path" ]] || { echo "Installed file missing: $path" >&2; exit 4; }
done
for exe in initialize_localization.py record_timing_diagnostics.sh scheduler_telemetry.py scene.py; do
  [[ -x "$full_prefix/lib/atlas_full_stack_demo/$exe" ]] || { echo "Executable not installed: $exe" >&2; exit 5; }
done
for f in src/atlas_full_stack_demo/scripts/initialize_localization.py src/atlas_full_stack_demo/scripts/scene.py; do
  PYTHONPYCACHEPREFIX="${TMPDIR:-/tmp}/atlas_pycache" python3 -m py_compile "$f"
done
printf 'SMOKE PASS: packages, launch/config files, scripts, and map checksums verified. No ROS graph started.\n'
