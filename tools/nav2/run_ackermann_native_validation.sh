#!/usr/bin/env bash
set -Eeuo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"
branch="$(git branch --show-current)"
if [[ "$branch" != feature/prd-gap-closure ]]; then
  echo "Expected development branch feature/prd-gap-closure; current branch is $branch." >&2
  exit 2
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble is required at /opt/ros/humble.' >&2
  exit 2
fi
# Start from the ROS underlay instead of inheriting stale stable or demo overlays.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH
unset PYTHONPATH ROS_PACKAGE_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/humble/setup.bash

overlay_install="${ATLAS_NAV2_OVERLAY:-$repo_root/.nav2_cache/ackermann/install}"
overlay_install="$(mkdir -p "$overlay_install" && cd "$overlay_install" && pwd -P)"
overlay_root="$(dirname "$overlay_install")"
build_base="$overlay_root/build"
evidence_parent="${ATLAS_VALIDATION_EVIDENCE_ROOT:-$HOME/validation_evidence/ackermann}"
evidence_root="$evidence_parent/native_$(date -u +%Y%m%dT%H%M%SZ)"
domain_start="${ATLAS_NAV2_DOMAIN_START:-20}"
if ! [[ "$domain_start" =~ ^[0-9]+$ ]] || (( domain_start < 0 || domain_start + 41 > 232 )); then
  echo 'ATLAS_NAV2_DOMAIN_START must be an integer in the range 0..191.' >&2
  exit 2
fi
mkdir -p "$evidence_parent"
mkdir "$evidence_root"

export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="$domain_start"
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
export ATLAS_NAV2_OVERLAY="$overlay_install"

required_ros_packages=(nav2_controller nav2_planner nav2_map_server nav2_bt_navigator
  nav2_lifecycle_manager nav2_smac_planner autoware_control_msgs)
for package in "${required_ros_packages[@]}"; do
  package_prefix="$(ros2 pkg prefix "$package" 2>/dev/null || true)"
  if [[ -z "$package_prefix" ]]; then
    echo "Missing ROS package $package. See docs/nav2/NATIVE_UBUNTU_VALIDATION.md." >&2
    exit 2
  fi
  resolved_ros_prefix="$(realpath "$package_prefix")"
  case "$resolved_ros_prefix" in
    /opt/ros/humble|/opt/ros/humble/*) ;;
    *)
      echo "$package resolved outside the official Humble underlay: $package_prefix" >&2
      exit 2
      ;;
  esac
done

printf 'Evidence directory: %s\nOverlay install: %s\n' "$evidence_root" "$overlay_install"

colcon build \
  --base-paths src/atlas_nav2_demo \
  --packages-select atlas_nav2_demo \
  --build-base "$build_base" \
  --install-base "$overlay_install" \
  --log-base "$evidence_root/colcon_build_logs" \
  --executor sequential \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON

source "$overlay_install/setup.bash"
resolved_prefix="$(ros2 pkg prefix atlas_nav2_demo)"
expected_prefix="$overlay_install/atlas_nav2_demo"
if [[ "$(realpath "$resolved_prefix")" != "$(realpath "$expected_prefix")" ]]; then
  echo "Development overlay did not take precedence: $resolved_prefix" >&2
  exit 2
fi

colcon test \
  --base-paths src/atlas_nav2_demo \
  --packages-select atlas_nav2_demo \
  --build-base "$build_base" \
  --install-base "$overlay_install" \
  --log-base "$evidence_root/colcon_test_logs" \
  --executor sequential
colcon test-result \
  --test-result-base "$build_base/atlas_nav2_demo" --verbose \
  | tee "$evidence_root/colcon_test_result.txt"

python3 tools/nav2/sample_ackermann_runtime.py \
  --output "$evidence_root/runtime_resources.jsonl" \
  --period 0.5 >"$evidence_root/runtime_sampler.log" 2>&1 &
sampler_pid=$!
stop_sampler() {
  if kill -0 "$sampler_pid" 2>/dev/null; then
    kill -TERM "$sampler_pid" 2>/dev/null || true
    wait "$sampler_pid" 2>/dev/null || true
  fi
}
trap stop_sampler EXIT INT TERM

run_navigation() {
  local scenario="$1" repetitions="$2" domain_offset="$3"
  local output="$evidence_root/navigation/$scenario"
  mkdir -p "$(dirname "$output")"
  ros2 run atlas_nav2_demo run_ackermann_acceptance.py \
    --scenario "$scenario" --repetitions "$repetitions" \
    --domain-start "$((domain_start + domain_offset))" \
    --evidence-root "$output"
}

run_navigation straight 5 0
run_navigation curved 5 5
run_navigation avoidable 5 10
run_navigation blocked 1 15

ros2 run atlas_nav2_demo validate_ackermann_faults.py \
  --fault all --domain-start "$((domain_start + 16))" \
  --evidence-root "$evidence_root/full_graph_faults"

python3 tools/nav2/run_ackermann_timing_campaign.py \
  --trials 20 --domain-start "$((domain_start + 22))" \
  --evidence-root "$evidence_root/cancellation_timing"

stop_sampler
trap - EXIT INT TERM
printf 'All configured checks passed. Evidence: %s\n' "$evidence_root"
