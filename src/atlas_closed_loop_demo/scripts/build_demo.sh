#!/usr/bin/env bash
# Audit the full closure, never install system packages, and build only missing packages.
set -eo pipefail
atlas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$atlas_root"
export ATLAS_REPO_ROOT="$atlas_root"
export ATLAS_AUTOWARE_ROOT="${ATLAS_AUTOWARE_ROOT:-$(cd "$atlas_root/.." && pwd)/references/autoware}"
trap 'python3 src/atlas_closed_loop_demo/scripts/build_status.py' EXIT
source /opt/ros/humble/setup.bash
# Keep this test's caches across reboot; reuse older overlays when present.
export ENABLE_AGNOCAST=0
for atlas_overlay in /tmp/atlas_map_upstream_install /tmp/atlas_map_demo_install /tmp/atlas_trajectory_upstream_install /tmp/atlas_closed_loop_upstream_install "$atlas_root/.closed_loop_cache/install"; do
  if [[ -f "$atlas_overlay/setup.bash" ]]; then source "$atlas_overlay/setup.bash"; fi
done
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j$CMAKE_BUILD_PARALLEL_LEVEL}"
python3 - <<'PY'
import json
import os
from pathlib import Path
import subprocess

ref = Path(os.environ['ATLAS_AUTOWARE_ROOT'])
for repo, commit in [(ref, '10718787ba6e28f038a0cb29ff99cc627b5abfd2'),
                     (ref / 'src/core/autoware_core', 'f25f83c632c1984ec276c894c41857d4abc0dad8'),
                     (ref / 'src/universe/autoware_universe', '6e477c645efec33f7909095eea684474e97f5e3d'),
                     (ref / 'src/core/autoware_simple_planning_simulator', '82d2c93a28a25f8802e8f5a21b4ec850beea9080')]:
    if subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip() != commit:
        raise SystemExit(f'Pinned source differs: {repo}')
bases = [str(ref / 'src' / s) for s in ['core', 'universe', 'external']]
targets = ['autoware_map_loader', 'autoware_map_projection_loader', 'autoware_lanelet2_map_visualizer', 'autoware_path_generator', 'autoware_planning_topic_converter', 'autoware_velocity_smoother', 'autoware_simple_planning_simulator', 'autoware_trajectory_follower_node', 'autoware_vehicle_cmd_gate']
listing = subprocess.check_output(['colcon', 'list', '--base-paths', *bases,
                                  '--packages-up-to', *targets], text=True)
rows = [line.split('\t') for line in listing.splitlines()]
# colcon --packages-up-to also includes test-only manifest edges. Build with
# BUILD_TESTING=OFF; audit the complete non-test dependency closure instead.
from catkin_pkg.package import parse_package
byname = {r[0]: r for r in rows}
needed = set()
def visit(name):
    if name in needed or name not in byname: return
    needed.add(name)
    pkg = parse_package(Path(byname[name][1]) / 'package.xml')
    pkg.evaluate_conditions(dict(os.environ))
    for tag in ['build_depends', 'buildtool_depends', 'build_export_depends', 'buildtool_export_depends', 'exec_depends']:
        for edge in getattr(pkg, tag):
            if edge.evaluated_condition: visit(edge.name)
for target in targets: visit(target)
rows = [r for r in rows if r[0] in needed]
listing = '\n'.join('\t'.join(r) for r in rows) + '\n'
audit = ['rosdep', 'check', '--rosdistro', 'humble', '--ignore-src']
audit += [f'--dependency-types={t}' for t in ['build', 'buildtool', 'build_export', 'buildtool_export', 'exec']]
audit += ['--from-paths', *[r[1] for r in rows]]
result = subprocess.run(audit, capture_output=True, text=True)
evidence = Path('src/atlas_closed_loop_demo/results/current_audit')
evidence.mkdir(parents=True, exist_ok=True)
(evidence / 'dependency_audit.log').write_text(result.stdout + result.stderr)
Path('/tmp/atlas_closed_loop_dependency_audit.log').write_text(result.stdout + result.stderr)
print(result.stdout + result.stderr, flush=True)
if result.returncode:
    raise SystemExit('Dependencies missing: obtain approval/install externally before resuming. Nothing was installed.')
prefixes = os.environ['AMENT_PREFIX_PATH'].split(':')
cached = [r[0] for r in rows if any(Path(p, 'share', r[0], 'package.xml').exists() for p in prefixes)]
missing = [r[0] for r in rows if r[0] not in cached]
(evidence / 'closure.tsv').write_text(listing)
(evidence / 'selection.json').write_text(json.dumps({'cached': cached, 'missing': missing}, indent=2))
Path('/tmp/atlas_closed_loop_closure.tsv').write_text(listing)
Path('/tmp/atlas_closed_loop_selection.json').write_text(json.dumps({'cached': cached, 'missing': missing}))
print('Reuse:', cached, '\nBuild:', missing, flush=True)
if missing:
    cache = Path(os.environ['ATLAS_REPO_ROOT']) / '.closed_loop_cache'
    subprocess.run(['python3', 'src/atlas_closed_loop_demo/scripts/colcon_sync_io.py', '--log-base', str(cache/'log'), 'build',
                    '--base-paths', *bases, '--packages-select', *missing,
                    '--packages-ignore', *sorted(set(byname) - needed),
                    '--packages-skip-build-finished', '--executor', 'sequential',
                    '--build-base', str(cache/'build'),
                    '--install-base', str(cache/'install'),
                    '--event-handlers', 'console_direct+', 'status-', '--cmake-args',
                    '-DBUILD_TESTING=OFF', '-DCMAKE_BUILD_TYPE=Release',
                    '-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=gold', '-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=gold'], check=True)
PY
source "$atlas_root/.closed_loop_cache/install/setup.bash"
python3 src/atlas_closed_loop_demo/scripts/colcon_sync_io.py --log-base "$atlas_root/.closed_loop_cache/demo_log" build \
  --base-paths src/atlas_map_integration_demo src/atlas_closed_loop_demo \
  --build-base "$atlas_root/.closed_loop_cache/demo_build" \
  --install-base "$atlas_root/.closed_loop_cache/demo_install" \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=gold -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=gold
