#!/usr/bin/env bash
# Audit the full closure, never install system packages, and build only missing packages.
set -eo pipefail
atlas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$atlas_root"
source src/atlas_full_stack_demo/scripts/environment.sh
for atlas_overlay in /tmp/atlas_map_upstream_install /tmp/atlas_map_demo_install; do
  [[ -f "$atlas_overlay/setup.bash" ]] && source "$atlas_overlay/setup.bash"
done
if [[ -f /tmp/atlas_trajectory_upstream_install/setup.bash ]]; then
  source /tmp/atlas_trajectory_upstream_install/setup.bash
fi
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
python3 - <<'PY'
import json
import os
from pathlib import Path
import subprocess

ref = Path(os.environ['ATLAS_AUTOWARE_ROOT'])
for repo, commit in [(ref, '10718787ba6e28f038a0cb29ff99cc627b5abfd2'),
                     (ref / 'src/core/autoware_core', 'f25f83c632c1984ec276c894c41857d4abc0dad8')]:
    if subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip() != commit:
        raise SystemExit(f'Pinned source differs: {repo}')
bases = [str(ref / 'src' / s) for s in ['core', 'universe', 'external']]
targets = ['autoware_path_generator', 'autoware_planning_topic_converter', 'autoware_velocity_smoother']
listing = subprocess.check_output(['colcon', 'list', '--base-paths', *bases,
                                  '--packages-up-to', *targets], text=True)
rows = [line.split('\t') for line in listing.splitlines()]
audit = ['rosdep', 'check', '--rosdistro', 'humble', '--ignore-src']
audit += [f'--dependency-types={t}' for t in ['build', 'buildtool', 'build_export', 'buildtool_export', 'exec']]
audit += ['--from-paths', *[r[1] for r in rows]]
result = subprocess.run(audit, capture_output=True, text=True)
Path('/tmp/atlas_trajectory_dependency_audit.log').write_text(result.stdout + result.stderr)
print(result.stdout + result.stderr, flush=True)
if result.returncode:
    raise SystemExit('Dependencies missing: obtain approval/install externally before resuming. Nothing was installed.')
prefixes = os.environ['AMENT_PREFIX_PATH'].split(':')
cached = [r[0] for r in rows if any(Path(p, 'share', r[0], 'package.xml').exists() for p in prefixes)]
missing = [r[0] for r in rows if r[0] not in cached]
Path('/tmp/atlas_trajectory_closure.tsv').write_text(listing)
Path('/tmp/atlas_trajectory_selection.json').write_text(json.dumps({'cached': cached, 'missing': missing}))
print('Reuse:', cached, '\nBuild:', missing, flush=True)
if missing:
    subprocess.run(['colcon', '--log-base', '/tmp/atlas_trajectory_upstream_log', 'build',
                    '--base-paths', *bases, '--packages-select', *missing,
                    '--packages-skip-build-finished', '--executor', 'sequential',
                    '--build-base', '/tmp/atlas_trajectory_upstream_build',
                    '--install-base', '/tmp/atlas_trajectory_upstream_install',
                    '--event-handlers', 'console_direct+', '--cmake-args',
                    '-DBUILD_TESTING=OFF', '-DCMAKE_BUILD_TYPE=Release'], check=True)
PY
source /tmp/atlas_trajectory_upstream_install/setup.bash
colcon --log-base /tmp/atlas_trajectory_demo_log build \
  --base-paths src/atlas_trajectory_generation_demo \
  --build-base /tmp/atlas_trajectory_demo_build \
  --install-base /tmp/atlas_trajectory_demo_install \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
