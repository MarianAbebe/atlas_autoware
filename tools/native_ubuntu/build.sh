#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"
export ATLAS_REPO_ROOT="$repo_root"
export ATLAS_AUTOWARE_ROOT="${ATLAS_AUTOWARE_ROOT:-$(cd "$repo_root/.." && pwd)/references/autoware}"
export ATLAS_MAP_DIR="${ATLAS_MAP_DIR:-$HOME/Downloads}"

"$repo_root/tools/native_ubuntu/preflight.sh"
"$repo_root/tools/native_ubuntu/fetch_autoware.sh"
source /opt/ros/humble/setup.bash
export ENABLE_AGNOCAST=0 CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1

# Build dependencies in the order expected by each package's established workflow.
bash src/atlas_map_integration_demo/scripts/build_demo.sh
bash src/atlas_closed_loop_demo/scripts/build_demo.sh
bash src/atlas_obstacle_stopping_demo/scripts/build_demo.sh
bash src/atlas_perception_planning_demo/scripts/build_demo.sh
bash src/atlas_localization_closed_loop_demo/scripts/build_demo.sh
bash src/atlas_full_stack_demo/scripts/build_demo.sh

source src/atlas_full_stack_demo/scripts/environment.sh
"$repo_root/tools/native_ubuntu/smoke_test.sh"
