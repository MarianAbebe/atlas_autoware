#!/usr/bin/env bash
set -eo pipefail

find_repo_root() {
  local current
  current="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
  while [[ "$current" != / ]]; do
    if [[ -f "$current/src/atlas_full_stack_demo/scripts/environment.sh" ]]; then
      printf '%s\n' "$current"
      return 0
    fi
    current="$(dirname "$current")"
  done
  return 1
}

if repo_root="$(find_repo_root)"; then
  cd "$repo_root"
  source "$repo_root/src/atlas_full_stack_demo/scripts/environment.sh"
elif ! command -v ros2 >/dev/null 2>&1; then
  echo "Cannot locate the Atlas repository or an existing ROS 2 environment." >&2
  exit 2
fi
set -u

if (( $# > 1 )); then
  echo "Usage: $0 [results-directory]" >&2
  exit 2
fi

results_dir="${1:-/tmp/atlas_freshness_controlled_$(date -u +%Y%m%d_%H%M%S)}"
if [[ -e "$results_dir" ]]; then
  echo "Refusing to overwrite existing path: $results_dir" >&2
  exit 2
fi
mkdir -p "$results_dir"
bag_dir="$results_dir/timing_bag"

topics=(
  /atlas_loc/diagnostics/scene_timing
  /atlas_loc/distortion_corrector/debug/pipeline_latency_ms
  /atlas_loc/euclidean_cluster/debug/pipeline_latency_ms
  /atlas_loc/ndt/exe_time_ms
  /atlas_loc/ndt/ndt_pose_with_covariance
  /atlas_loc/ekf/ekf_odom
  /atlas_loc/base_safety_status
  /atlas_loc/safety_status
  /atlas_loc/sim/output/odometry
  /diagnostics
  /atlas_loc/raw_trajectory
  /atlas_loc/obstacle_trajectory
  /atlas_loc/trajectory
  /atlas_loc/base_safety_heartbeat
  /atlas_loc/control_cmd
)

echo "Recording Atlas timing diagnostics."
echo "Output directory: $results_dir"
echo "Press Ctrl+C once to stop and finalize the rosbag."

bag_pid=""
stop_recorder() {
  trap - INT TERM
  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid"
  fi
  wait "$bag_pid" || true
  exit 130
}
trap stop_recorder INT TERM

ros2 bag record --output "$bag_dir" "${topics[@]}" &
bag_pid=$!
wait "$bag_pid"
status=$?
trap - INT TERM
exit "$status"
