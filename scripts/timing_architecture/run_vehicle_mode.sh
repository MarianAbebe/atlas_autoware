#!/usr/bin/env bash
# Isolated pinned vehicle component. Real-time default; opt-in fixed 20 ms steps.
set -eo pipefail
atlas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$atlas_root"
mode=realtime
case "${1:-}" in
  --fixed-step) mode=controlled; shift ;;
  --realtime) shift ;;
  '') ;;
  *) echo 'Usage: run_vehicle_mode.sh [--fixed-step|--realtime] [stall-ms (0..1000)]' >&2; exit 2 ;;
esac
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
source .closed_loop_cache/demo_install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-85}"
cmake -S scripts/timing_architecture -B .localization_cache/timing_build -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build .localization_cache/timing_build -j1 >/dev/null
exec .localization_cache/timing_build/pinned_step_probe "$mode" "${1:-500}"
