#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mode="${1:---check}"
case "$mode" in --check|--build|--smoke|--faults|--repeatability|--all) ;; *)
  echo "Usage: $0 {--check|--build|--smoke|--faults|--repeatability|--all}" >&2; exit 2;; esac

evidence_root="${ATLAS_VALIDATION_EVIDENCE_ROOT:-$HOME/validation_evidence/autoware_ackermann_native}"
overlay_root="${ATLAS_AUTOWARE_OVERLAY:-$HOME/roam/overlays/atlas_autoware_native}"
autoware_setup="${ATLAS_AUTOWARE_SETUP:-}"
map_dir="${ATLAS_MAP_DIR:-$HOME/atlas_assets}"
domain_start="${ATLAS_VALIDATION_DOMAIN_START:-120}"
[[ "$domain_start" =~ ^[0-9]+$ ]] || { echo "ATLAS_VALIDATION_DOMAIN_START must be an integer" >&2; exit 2; }
(( domain_start >= 1 && domain_start + 10 <= 232 )) || { echo "ROS domain range must fit within 1..232" >&2; exit 2; }

failures=0
check() { local label="$1"; shift; if "$@"; then printf 'OK      %s\n' "$label"; else printf 'MISSING %s\n' "$label"; failures=$((failures+1)); fi; }
ros2_pkg_present() { ros2 pkg prefix "$1" >/dev/null 2>&1; }
source_setup() {
  local setup_path="$1" had_nounset=0 source_status=0
  case $- in *u*) had_nounset=1 ;; esac
  set +u
  source "$setup_path" || source_status=$?
  if (( had_nounset )); then set -u; fi
  return "$source_status"
}

check_environment() {
  [[ -f /etc/os-release ]] || return 1
  . /etc/os-release
  [[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo "Requires Ubuntu 22.04; found ${PRETTY_NAME:-unknown}" >&2; return 1; }
  [[ "$(uname -m)" == x86_64 ]] || { echo "Native validation target is x86_64; found $(uname -m)" >&2; return 1; }
  [[ -r /opt/ros/humble/setup.bash ]] || { echo "Missing /opt/ros/humble/setup.bash" >&2; return 1; }
  source_setup /opt/ros/humble/setup.bash
  command -v colcon >/dev/null && command -v ros2 >/dev/null && command -v timeout >/dev/null || return 1
  [[ -n "$autoware_setup" && -r "$autoware_setup" ]] || {
    echo "Set ATLAS_AUTOWARE_SETUP to the pinned Autoware dependency overlay setup.bash" >&2; return 1; }
  [[ -d "${ATLAS_AUTOWARE_ROOT:-}" ]] || { echo "Set ATLAS_AUTOWARE_ROOT to the pinned Autoware 1.9.0 checkout" >&2; return 1; }
  for file in finalMap_repaired.osm map_projector_info.yaml pointcloud_map_georeferenced.pcd; do
    [[ -s "$map_dir/$file" ]] || { echo "Missing map asset $map_dir/$file" >&2; return 1; }
  done
  (cd "$map_dir" && sha256sum -c "$repo_root/tools/native_ubuntu/map_assets.sha256") || {
    echo "Map asset checksum verification failed" >&2; return 1; }
  local required=(git python3 rosdep vcs cmake)
  local bin
  for bin in "${required[@]}"; do command -v "$bin" >/dev/null || { echo "Missing command: $bin" >&2; return 1; }; done
}

prepare_ros() {
  source_setup /opt/ros/humble/setup.bash
  source_setup "$autoware_setup"
  export ATLAS_REPO_ROOT="$repo_root" ATLAS_AUTOWARE_ROOT ATLAS_MAP_DIR="$map_dir"
  export ENABLE_AGNOCAST=0 CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
  export ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
  mkdir -p "$evidence_root"
}

run_build() {
  prepare_ros
  local build_base="${ATLAS_BUILD_BASE:-$overlay_root/build}"
  local install_base="${ATLAS_INSTALL_BASE:-$overlay_root/install}"
  mkdir -p "$overlay_root"
  local simulator_source="$overlay_root/src/autoware_simple_planning_simulator"
  if [[ ! -d "$simulator_source" ]]; then
    mkdir -p "$(dirname "$simulator_source")"
    python3 "$repo_root/tools/prepare_instrumented_simulator.py" \
      --autoware-root "$ATLAS_AUTOWARE_ROOT" --output "$simulator_source"
  fi
  local cluster_source="$ATLAS_AUTOWARE_ROOT/src/core/autoware_core/perception/autoware_euclidean_cluster_object_detector"
  [[ -f "$cluster_source/CMakeLists.txt" ]] || { echo "Pinned Euclidean cluster package missing: $cluster_source" >&2; return 1; }
  cmake -S "$cluster_source" -B "$overlay_root/cluster_build" \
    -DCMAKE_INSTALL_PREFIX="$overlay_root/cluster_install" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
  cmake --build "$overlay_root/cluster_build" -j1
  cmake --install "$overlay_root/cluster_build"
  export AMENT_PREFIX_PATH="$overlay_root/cluster_install${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$overlay_root/cluster_install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  colcon build --base-paths \
    "$simulator_source" \
    "$repo_root/src/atlas_vehicle_interface" \
    "$repo_root/src/atlas_map_integration_demo" \
    "$repo_root/src/atlas_closed_loop_demo" \
    "$repo_root/src/atlas_trajectory_generation_demo" \
    "$repo_root/src/atlas_obstacle_stopping_demo" \
    "$repo_root/src/atlas_perception_planning_demo" \
    "$repo_root/src/atlas_localization_closed_loop_demo" \
    "$repo_root/src/atlas_full_stack_demo" \
    "$repo_root/src/atlas_sensor_kit" \
    "$repo_root/src/atlas_distortion_corrector_test" \
    --packages-select autoware_simple_planning_simulator atlas_vehicle_interface atlas_map_integration_demo \
      atlas_closed_loop_demo atlas_trajectory_generation_demo atlas_obstacle_stopping_demo \
      atlas_perception_planning_demo atlas_localization_closed_loop_demo atlas_full_stack_demo \
      atlas_sensor_kit atlas_distortion_corrector_test \
    --executor sequential --build-base "$build_base" --install-base "$install_base" \
    --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
  colcon test --base-paths \
    "$simulator_source" \
    "$repo_root/src/atlas_vehicle_interface" \
    "$repo_root/src/atlas_map_integration_demo" \
    "$repo_root/src/atlas_closed_loop_demo" \
    "$repo_root/src/atlas_trajectory_generation_demo" \
    "$repo_root/src/atlas_obstacle_stopping_demo" \
    "$repo_root/src/atlas_perception_planning_demo" \
    "$repo_root/src/atlas_localization_closed_loop_demo" \
    "$repo_root/src/atlas_full_stack_demo" \
    "$repo_root/src/atlas_sensor_kit" \
    "$repo_root/src/atlas_distortion_corrector_test" \
    --packages-select autoware_simple_planning_simulator atlas_vehicle_interface atlas_map_integration_demo \
      atlas_closed_loop_demo atlas_trajectory_generation_demo atlas_obstacle_stopping_demo \
      atlas_perception_planning_demo atlas_localization_closed_loop_demo atlas_full_stack_demo \
      atlas_sensor_kit atlas_distortion_corrector_test \
    --executor sequential --build-base "$build_base" --install-base "$install_base" \
    --ctest-args -j1
  colcon test-result --test-result-base "$build_base" --verbose
  source_setup "$install_base/setup.bash"
  [[ "$(ros2 pkg prefix atlas_full_stack_demo)" == "$install_base/atlas_full_stack_demo" ]] || {
    echo "Development overlay did not take precedence for atlas_full_stack_demo" >&2; return 1; }
}

require_overlay() {
  [[ -r "$overlay_root/install/setup.bash" ]] || {
    echo "Atlas overlay missing. Run --build or set ATLAS_AUTOWARE_OVERLAY to a built overlay." >&2; return 1; }
}

run_lap() {
  local laps="$1" label="$2" seconds="$3" domain="$4" output="$evidence_root/$label"
  [[ ! -e "$output" ]] || { echo "Evidence path already exists: $output" >&2; return 1; }
  export ROS_DOMAIN_ID="$domain"
  mkdir -p "$output"
  printf 'RUN %s laps=%s domain=%s timeout=%ss evidence=%s\n' "$label" "$laps" "$domain" "$seconds" "$output"
  timeout --signal=INT --kill-after=15 "$((seconds+20))" \
    python3 "$repo_root/src/atlas_full_stack_demo/scripts/run_lap_validation.py" \
      --laps "$laps" --timeout "$seconds" --output "$output" --steering-step-diagnostics
}

run_faults() {
  local domain="$1" output="$evidence_root/full_graph_estop"
  [[ ! -e "$output" ]] || { echo "Evidence path already exists: $output" >&2; return 1; }
  export ROS_DOMAIN_ID="$domain"
  echo "RUN full-graph E-stop/clear/no-auto-resume"
  timeout --signal=INT --kill-after=15 260 \
    python3 "$repo_root/src/atlas_full_stack_demo/scripts/run_estop_fault_validation.py" \
      --laps 1 --timeout 240 --output "$output"
  local scenario i=1
  for scenario in loss localization_loss; do
    output="$evidence_root/full_graph_${scenario}"
    [[ ! -e "$output" ]] || { echo "Evidence path already exists: $output" >&2; return 1; }
    export ROS_DOMAIN_ID="$((domain+i))"
    echo "RUN full-graph $scenario (bounded source/localization-loss stop)"
    timeout --signal=INT --kill-after=15 110 \
      python3 "$repo_root/src/atlas_full_stack_demo/scripts/run_validation.py" \
        --scenario "$scenario" --timeout 80 --output "$output"
    i=$((i+1))
  done
}

if ! check_environment; then
  echo "Prerequisite check failed. This script does not install packages or alter the system." >&2
  exit 2
fi
echo "OK      Ubuntu 22.04 x86_64, ROS Humble, pinned Autoware setup, and required map assets"
if [[ "$mode" == --check ]]; then
  set +u
  source_setup /opt/ros/humble/setup.bash
  source_setup "$autoware_setup"
  for pkg in autoware_control_msgs autoware_simple_planning_simulator autoware_ndt_scan_matcher autoware_euclidean_cluster_object_detector; do
    check "ROS package $pkg" ros2_pkg_present "$pkg"
  done
  (( failures == 0 )) || exit 2
  echo "CHECK PASS. No packages installed, workspace built, or ROS graph started."
  exit 0
fi

case "$mode" in
  --build) run_build ;;
  --smoke)
    require_overlay; prepare_ros; source_setup "$overlay_root/install/setup.bash"
    run_lap 1 "smoke_$(date -u +%Y%m%dT%H%M%SZ)" 180 "$domain_start" ;;
  --faults)
    require_overlay; prepare_ros; source_setup "$overlay_root/install/setup.bash"
    run_faults "$domain_start" ;;
  --repeatability)
    require_overlay; prepare_ros; source_setup "$overlay_root/install/setup.bash"
    for i in 0 1 2 3 4; do
      run_lap 1 "one_lap_repeat_$((i+1))_$(date -u +%Y%m%dT%H%M%SZ)" 200 "$((domain_start+i))"
    done
    run_lap 5 "continuous_five_lap_$(date -u +%Y%m%dT%H%M%SZ)" 650 "$((domain_start+5))" ;;
  --all)
    run_build
    source_setup "$overlay_root/install/setup.bash"
    run_lap 1 "smoke_$(date -u +%Y%m%dT%H%M%SZ)" 180 "$domain_start"
    run_faults "$((domain_start+1))"
    for i in 0 1 2 3 4; do
      run_lap 1 "one_lap_repeat_$((i+1))_$(date -u +%Y%m%dT%H%M%SZ)" 200 "$((domain_start+2+i))"
    done
    run_lap 5 "continuous_five_lap_$(date -u +%Y%m%dT%H%M%SZ)" 650 "$((domain_start+7))" ;;
esac
