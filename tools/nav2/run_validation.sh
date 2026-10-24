#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"
set +u
source /opt/ros/humble/setup.bash
source .nav2_cache/install/setup.bash
set -u
output="${1:-/tmp/atlas_nav2_validation_$(date -u +%Y%m%d_%H%M%S)}"
exec ros2 run atlas_nav2_demo validate_closed_loop.py --output "$output"
