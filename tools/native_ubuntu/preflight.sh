#!/usr/bin/env bash
set -uo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
map_dir="${ATLAS_MAP_DIR:-$HOME/atlas_assets}"
ref_root="${ATLAS_AUTOWARE_ROOT:-$(cd "$repo_root/.." && pwd)/references/autoware}"
errors=0
warnings=0
ok() { printf 'OK      %s\n' "$1"; }
warn() { printf 'WARN    %s\n' "$1"; warnings=$((warnings+1)); }
fail() { printf 'MISSING %s\n' "$1"; errors=$((errors+1)); }

if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] && ok "Ubuntu 22.04" || fail "Ubuntu 22.04 required; detected ${PRETTY_NAME:-unknown}"
else fail "Cannot read /etc/os-release"; fi
arch="$(uname -m)"
if [[ "$arch" == x86_64 ]]; then ok "CPU architecture x86_64"; else warn "CPU architecture is $arch; target release is x86_64 and this is not an x86_64 validation"; fi
ram_kib="$(awk '/MemTotal:/ {print $2}' /proc/meminfo 2>/dev/null)"
if [[ -n "$ram_kib" ]]; then
  ram_gib=$((ram_kib / 1048576))
  (( ram_gib >= 8 )) && ok "RAM ${ram_gib} GiB" || warn "RAM ${ram_gib} GiB; 8 GiB recommended for a clean source build"
fi
free_kib="$(df -Pk "$repo_root" | awk 'NR==2 {print $4}')"
free_gib=$((free_kib / 1048576))
(( free_gib >= 50 )) && ok "Free disk ${free_gib} GiB" || warn "Free disk ${free_gib} GiB; reserve at least 50 GiB before importing/building Autoware"
(( $(nproc) >= 6 )) && ok "CPUs $(nproc)" || warn "Only $(nproc) CPUs; builds are sequential and will take longer"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  had_nounset=0
  case $- in *u*) had_nounset=1 ;; esac
  set +u
  # shellcheck disable=SC1091
  if source /opt/ros/humble/setup.bash; then source_status=0; else source_status=$?; fi
  if (( had_nounset )); then set -u; else set +u; fi
  (( source_status == 0 )) || fail "ROS 2 Humble setup failed"
  command -v ros2 >/dev/null && ok "ROS 2 Humble" || fail "ROS setup exists but ros2 is unavailable"
else fail "ROS 2 Humble not installed at /opt/ros/humble"; fi
for bin in git python3 colcon rosdep vcs cmake; do command -v "$bin" >/dev/null 2>&1 && ok "$bin: $(command -v "$bin")" || fail "$bin not installed"; done
if command -v ld.gold >/dev/null 2>&1; then ok "ld.gold: $(command -v ld.gold)"; else fail "ld.gold missing; install binutils-gold (established Atlas colcon scripts pass -fuse-ld=gold)"; fi
python3 -c 'import sys; assert sys.version_info[:2] == (3,10), sys.version; print("OK      Python", sys.version.split()[0])' || fail "Python 3.10 required for Ubuntu 22.04 ROS Humble"
python3 -c 'import catkin_pkg' >/dev/null 2>&1 && ok "python3 catkin_pkg" || fail "python3-catkin-pkg not installed"
if command -v glxinfo >/dev/null 2>&1; then glxinfo -B 2>/dev/null | awk '/OpenGL vendor|OpenGL renderer|OpenGL version/ {print "OK      "$0}'; else warn "glxinfo unavailable; RViz OpenGL renderer cannot be inspected (no package will be installed)"; fi

for file in finalMap_repaired.osm map_projector_info.yaml pointcloud_map_georeferenced.pcd; do
  path="$map_dir/$file"
  [[ -s "$path" ]] && ok "Map asset $path" || fail "Map asset missing: $path"
done
if [[ -s "$map_dir/finalMap_repaired.osm" && -s "$map_dir/map_projector_info.yaml" && -s "$map_dir/pointcloud_map_georeferenced.pcd" ]]; then
  (cd "$map_dir" && sha256sum -c "$repo_root/tools/native_ubuntu/map_assets.sha256") || fail "Map asset checksum mismatch"
fi
if [[ -d "$ref_root/.git" ]]; then
  root_rev="$(git -C "$ref_root" rev-parse HEAD 2>/dev/null || true)"
  [[ "$root_rev" == 10718787ba6e28f038a0cb29ff99cc627b5abfd2 ]] && ok "Autoware root 1.9.0 pin" || fail "Autoware checkout missing or root pin differs at $ref_root"
else warn "Pinned Autoware checkout not present at $ref_root; run fetch_autoware.sh after reviewing network access"; fi

printf '\nPreflight: %d blocking issue(s), %d warning(s).\n' "$errors" "$warnings"
(( errors == 0 ))
