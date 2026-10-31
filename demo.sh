#!/usr/bin/env bash
# One-command, visualization-only wrapper for the existing Atlas demo stack.
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./demo.sh clear      Run the engaged clear-route demo with lightweight RViz.
  ./demo.sh obstacle   Run the engaged obstacle-stop demo with lightweight RViz.
  ./demo.sh check      Check overlays, map assets, ROS tools, and domain availability.
  ./demo.sh help       Show this help.

Set ATLAS_DEMO_INSTALL to the focused install path produced by
docs/handoff/DEMO_GUIDE.md before running clear or obstacle.
Press Ctrl+C to stop only processes started by this script.
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
note() { printf '[ROAM demo] %s\n' "$*"; }

COMMAND="${1:-help}"
case "$COMMAND" in
  clear|obstacle|check|help) ;;
  *) usage >&2; fail "unknown command: $COMMAND" ;;
esac

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$REPO_ROOT"
REQUESTED_MAP_DIR="${ATLAS_MAP_DIR:-}"

if [[ "$COMMAND" == help ]]; then usage; exit 0; fi

ENV_SETUP="$REPO_ROOT/src/atlas_full_stack_demo/scripts/environment.sh"
[[ -r "$ENV_SETUP" ]] || fail "ROS environment setup is missing: $ENV_SETUP"
# The setup helper sources ROS/colcon scripts that may inspect unset variables.
set +u
source "$ENV_SETUP"
set -u

[[ "${ROS_DISTRO:-}" == humble ]] || fail "expected ROS 2 Humble; ROS_DISTRO=${ROS_DISTRO:-unset}"
[[ "${ROS_DOMAIN_ID:-}" == 86 ]] || fail "environment.sh must select ROS_DOMAIN_ID=86; got ${ROS_DOMAIN_ID:-unset}"

MAP_DIR="${REQUESTED_MAP_DIR:-/home/marian/Downloads}"
export ATLAS_MAP_DIR="$MAP_DIR"
ATLAS_DEMO_INSTALL="${ATLAS_DEMO_INSTALL:-}"
if [[ -n "$ATLAS_DEMO_INSTALL" ]]; then
  ATLAS_DEMO_INSTALL="$(realpath -m -- "$ATLAS_DEMO_INSTALL")"
  [[ -d "$ATLAS_DEMO_INSTALL" ]] || fail "ATLAS_DEMO_INSTALL does not exist: $ATLAS_DEMO_INSTALL"
  export ATLAS_DEMO_INSTALL
  export AMENT_PREFIX_PATH="$ATLAS_DEMO_INSTALL${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
fi

required_commands=(ros2 python3 flock timeout setsid)
for tool in "${required_commands[@]}"; do command -v "$tool" >/dev/null || fail "required command not found: $tool"; done
[[ -n "${ATLAS_DEMO_INSTALL:-}" ]] || fail "set ATLAS_DEMO_INSTALL to the focused install overlay documented in docs/handoff/DEMO_GUIDE.md"

DEMO_SHARE="$ATLAS_DEMO_INSTALL/share/atlas_full_stack_demo"
FULL_STACK_LAUNCH="$DEMO_SHARE/launch/full_stack.launch.py"
VIS_LAUNCH="$DEMO_SHARE/launch/demo_visualization.launch.py"
LIGHT_RVIZ="$DEMO_SHARE/config/demo_resource_limited.rviz"
MANUAL_START="$ATLAS_DEMO_INSTALL/lib/atlas_full_stack_demo/manual_start.py"
for path in "$FULL_STACK_LAUNCH" "$VIS_LAUNCH" "$LIGHT_RVIZ" "$MANUAL_START"; do
  [[ -r "$path" ]] || fail "required installed demo resource is missing: $path (rebuild atlas_full_stack_demo as documented)"
done

[[ "$(ros2 pkg prefix atlas_full_stack_demo 2>/dev/null || true)" == "$ATLAS_DEMO_INSTALL" ]] || fail "atlas_full_stack_demo does not resolve from ATLAS_DEMO_INSTALL; check overlay precedence"
for package in atlas_map_integration_demo atlas_closed_loop_demo atlas_localization_closed_loop_demo atlas_perception_planning_demo autoware_sample_vehicle_description; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || fail "ROS package is unavailable: $package (source the documented dependency overlays)"
done
command -v rviz2 >/dev/null || fail "rviz2 is unavailable in the sourced ROS environment"
ros2 pkg prefix robot_state_publisher >/dev/null 2>&1 || fail "robot_state_publisher is unavailable"
[[ -d "${ATLAS_AUTOWARE_ROOT:?ATLAS_AUTOWARE_ROOT was not set by environment.sh}/src/core/autoware_core" ]] || fail "pinned Autoware checkout is unavailable at $ATLAS_AUTOWARE_ROOT"

for asset in finalMap_repaired.osm pointcloud_map_georeferenced.pcd map_projector_info.yaml; do
  [[ -r "$MAP_DIR/$asset" ]] || fail "map asset missing: $MAP_DIR/$asset (set ATLAS_MAP_DIR to the map asset directory)"
done

if [[ "$COMMAND" == check ]]; then
  [[ -n "${DISPLAY:-}" ]] || fail "DISPLAY is unset; use a graphical desktop session for the live RViz demo"
  printf 'Environment: ROS %s, ROS_DOMAIN_ID=%s\n' "$ROS_DISTRO" "$ROS_DOMAIN_ID"
  printf 'Repository: %s\nDemo overlay: %s\n' "$REPO_ROOT" "$ATLAS_DEMO_INSTALL"
  printf 'Full-stack launch: %s\nVisualization launch: %s\nRViz config: %s\n' "$FULL_STACK_LAUNCH" "$VIS_LAUNCH" "$LIGHT_RVIZ"
  printf 'Map assets: %s\n' "$MAP_DIR"
  printf 'Packages: atlas_full_stack_demo=%s\n' "$(ros2 pkg prefix atlas_full_stack_demo)"
  printf 'Packages: atlas_map_integration_demo=%s atlas_closed_loop_demo=%s autoware_sample_vehicle_description=%s\n' \
    "$(ros2 pkg prefix atlas_map_integration_demo)" "$(ros2 pkg prefix atlas_closed_loop_demo)" "$(ros2 pkg prefix autoware_sample_vehicle_description)"
  printf 'Engagement helper: %s\n' "$MANUAL_START"
  timeout 30s ros2 launch atlas_full_stack_demo full_stack.launch.py --show-args >/dev/null || fail "full-stack launch description cannot be resolved"
  timeout 30s ros2 launch atlas_full_stack_demo demo_visualization.launch.py --show-args >/dev/null || fail "visualization launch description cannot be resolved"
  note 'Prerequisites look available. No simulation was launched.'
  exit 0
fi

[[ -n "${DISPLAY:-}" ]] || fail "DISPLAY is unset; run from the graphical Ubuntu desktop session"
SCENARIO="$COMMAND"
OBSTACLE=false
[[ "$SCENARIO" == obstacle ]] && OBSTACLE=true

# One script-managed session per domain. The topic check below also catches
# already-running demos started outside this script.
LOCK_FILE="${TMPDIR:-/tmp}/roam_atlas_demo_domain${ROS_DOMAIN_ID}.lock"
exec 9>"$LOCK_FILE"
flock -n 9 || fail "another demo.sh session owns ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

active_publisher_count() {
  local report
  report="$(ros2 topic info "$1" --no-daemon 2>/dev/null)" || return 1
  grep -Eo 'Publisher count:[[:space:]]*[0-9]+' <<<"$report" | awk '{print $3}' | tail -n 1
}

for topic in /atlas_loc/lidar/raw /atlas_loc/ekf/ekf_odom; do
  count="$(active_publisher_count "$topic" || true)"
  if [[ "${count:-0}" =~ ^[0-9]+$ ]] && (( count > 0 )); then
    fail "a publisher already exists on $topic in domain $ROS_DOMAIN_ID; stop the existing demo before starting another"
  fi
done

RESULTS_ROOT="$HOME/roam/demo_results"
mkdir -p "$RESULTS_ROOT"
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
RESULTS_DIR="$RESULTS_ROOT/${stamp}_${SCENARIO}_$$"
mkdir -- "$RESULTS_DIR" || fail "could not create unique results directory: $RESULTS_DIR"
mkdir -- "$RESULTS_DIR/validation"
printf 'scenario=%s\nstarted_utc=%s\nrepository=%s\nhead=%s\nros_domain_id=%s\nmap_dir=%s\ndemo_install=%s\nrviz_config=%s\n' \
  "$SCENARIO" "$stamp" "$REPO_ROOT" "$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || printf unknown)" \
  "$ROS_DOMAIN_ID" "$MAP_DIR" "$ATLAS_DEMO_INSTALL" "$LIGHT_RVIZ" >"$RESULTS_DIR/run_info.txt"

SIM_PID=""
RVIZ_PID=""
START_PID=""
SIM_RC="not_exited"
RVIZ_RC="not_started"
START_RC="not_started"
cleaning=0
stop_group() {
  local pid="$1" label="$2" attempt prior_rc
  [[ -n "$pid" ]] || return 0
  if kill -0 -- "-$pid" 2>/dev/null; then
    note "Stopping $label (process group $pid)"
    kill -TERM -- "-$pid" 2>/dev/null || true
    for attempt in {1..8}; do
      kill -0 -- "-$pid" 2>/dev/null || break
      sleep 1
    done
    kill -0 -- "-$pid" 2>/dev/null && kill -KILL -- "-$pid" 2>/dev/null || true
  fi
  case "$label" in
    SIM) prior_rc="$SIM_RC" ;;
    RVIZ) prior_rc="$RVIZ_RC" ;;
    START) prior_rc="$START_RC" ;;
  esac
  if [[ "$prior_rc" == not_exited || "$prior_rc" == not_started ]]; then
    if wait "$pid" 2>/dev/null; then prior_rc=0; else prior_rc=$?; fi
    printf -v "${label}_RC" '%s' "$prior_rc"
  fi
}
cleanup() {
  local status=$?
  (( cleaning )) && return
  cleaning=1
  trap - EXIT INT TERM
  stop_group "$START_PID" START
  stop_group "$RVIZ_PID" RVIZ
  stop_group "$SIM_PID" SIM
  {
    printf 'simulation_exit=%s\n' "$SIM_RC"
    printf 'visualization_exit=%s\n' "$RVIZ_RC"
    printf 'engagement_helper_exit=%s\n' "$START_RC"
    printf 'launcher_exit=%s\n' "$status"
    printf 'ended_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  } >"$RESULTS_DIR/exit_codes.txt"
  note "Results saved in $RESULTS_DIR"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

note "Starting $SCENARIO scenario (obstacle:=$OBSTACLE, request_engagement:=true)"
note "Domain $ROS_DOMAIN_ID; results: $RESULTS_DIR"
setsid ros2 launch atlas_full_stack_demo full_stack.launch.py \
  "obstacle:=$OBSTACLE" request_engagement:=true "results_dir:=$RESULTS_DIR/validation" \
  >"$RESULTS_DIR/simulation.log" 2>&1 &
SIM_PID=$!
printf '%s\n' "$SIM_PID" >"$RESULTS_DIR/simulation.pid"

# Wait for scene initialization, truth, and raw sensor data before RViz.
if ! kill -0 "$SIM_PID" 2>/dev/null; then
  if wait "$SIM_PID"; then SIM_RC=0; else SIM_RC=$?; fi
  fail "full-stack launch exited during startup (code $SIM_RC); see $RESULTS_DIR/simulation.log"
fi
if timeout --signal=TERM --kill-after=2s 120s python3 - >"$RESULTS_DIR/readiness.log" 2>&1 <<'PY'
import time
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('atlas_demo_readiness_probe')
seen = set()
def mark(name):
    return lambda _msg: seen.add(name)
node.create_subscription(PointCloud2, '/atlas_loc/lidar/raw', mark('raw'), qos_profile_sensor_data)
node.create_subscription(Odometry, '/atlas_loc/sim/output/odometry', mark('truth'), 10)
scene_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String, '/atlas_loc/scene_info', mark('scene'), scene_qos)
deadline = time.monotonic() + 45.0
try:
    while time.monotonic() < deadline and len(seen) < 3:
        rclpy.spin_once(node, timeout_sec=0.2)
    print('received=' + ','.join(sorted(seen)), flush=True)
    if seen != {'raw', 'truth', 'scene'}:
        raise SystemExit(1)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
then
  ready=1
else
  ready=0
fi
(( ready )) || fail "simulation did not deliver scene info, truth odometry, and raw LiDAR within 45 seconds; see $RESULTS_DIR/simulation.log and readiness.log"

note 'Starting the existing localization activation and engagement helper.'
setsid timeout --signal=TERM --kill-after=5s 65s ros2 run atlas_full_stack_demo manual_start.py \
  >"$RESULTS_DIR/manual_start.log" 2>&1 &
START_PID=$!
printf '%s\n' "$START_PID" >"$RESULTS_DIR/manual_start.pid"
while kill -0 "$START_PID" 2>/dev/null; do
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    fail "simulation exited while waiting for engagement; inspect simulation.log"
  fi
  sleep 1
done
if wait "$START_PID"; then START_RC=0; else START_RC=$?; fi
if (( START_RC != 0 )); then
  tail -n 20 "$RESULTS_DIR/manual_start.log" >&2 || true
  fail "localization/engagement helper failed (code $START_RC); see $RESULTS_DIR/manual_start.log"
fi
note 'The helper confirmed engagement and simulated motion; starting the lightweight RViz view.'
setsid ros2 launch atlas_full_stack_demo demo_visualization.launch.py "rviz_config:=$LIGHT_RVIZ" \
  >"$RESULTS_DIR/visualization.log" 2>&1 &
RVIZ_PID=$!
printf '%s\n' "$RVIZ_PID" >"$RESULTS_DIR/visualization.pid"
note 'RViz is open with the resource-limited configuration.'

# Keep both processes supervised. Closing RViz or a launch failure stops the
# other process rather than leaving an orphaned simulation.
while :; do
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    if wait "$SIM_PID"; then SIM_RC=0; else SIM_RC=$?; fi
    fail "full-stack launch exited unexpectedly (code $SIM_RC); see $RESULTS_DIR/simulation.log"
  fi
  if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
    if wait "$RVIZ_PID"; then RVIZ_RC=0; else RVIZ_RC=$?; fi
    fail "visualization launch exited (code $RVIZ_RC); simulation will be stopped"
  fi
  sleep 1
done
