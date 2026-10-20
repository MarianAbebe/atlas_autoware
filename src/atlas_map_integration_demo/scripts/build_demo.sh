#!/usr/bin/env bash
# Build only missing pinned packages; never invoke a package installer.
set -eo pipefail
cd /home/marian/roam/atlas_autoware
source /opt/ros/humble/setup.bash
for atlas_overlay in /tmp/atlas_deskew_deps_install /tmp/atlas_ndt_install /tmp/atlas_vehicle_sim_install /tmp/atlas_map_upstream_install; do
  if [[ -f "$atlas_overlay/setup.bash" ]]; then
    source "$atlas_overlay/setup.bash"
  fi
done
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
python3 - <<'PY'
import json
import os
from pathlib import Path
import subprocess

reference = Path('/home/marian/roam/references/autoware')
pins = {reference: '10718787ba6e28f038a0cb29ff99cc627b5abfd2',
        reference / 'src/core/autoware_core': 'f25f83c632c1984ec276c894c41857d4abc0dad8',
        reference / 'src/core/autoware_lanelet2_extension': '4a3420d8cc19906e7739618f8a1686400f79b4ac'}
for repo, commit in pins.items():
    actual = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != commit:
        raise SystemExit(f'Pinned source differs: {repo}: {actual}, expected {commit}')
bases = [str(reference / 'src' / x) for x in ['core', 'universe', 'external']]
targets = ['autoware_map_loader', 'autoware_map_projection_loader', 'autoware_lanelet2_map_visualizer']
listing = subprocess.check_output(['colcon', 'list', '--base-paths', *bases,
                                  '--packages-up-to', *targets], text=True)
rows = [line.split('\t') for line in listing.splitlines()]
audit = ['rosdep', 'check', '--rosdistro', 'humble', '--ignore-src']
audit += [f'--dependency-types={t}' for t in ['build', 'buildtool', 'build_export', 'buildtool_export', 'exec']]
audit += ['--from-paths', *[r[1] for r in rows], 'src/atlas_map_integration_demo']
subprocess.run(audit, check=True)
prefixes = os.environ['AMENT_PREFIX_PATH'].split(':')
cached = [r[0] for r in rows if any(Path(p, 'share', r[0], 'package.xml').exists() for p in prefixes)]
missing = [r[0] for r in rows if r[0] not in cached]
Path('/tmp/atlas_map_closure.tsv').write_text(listing)
Path('/tmp/atlas_map_build_selection.json').write_text(json.dumps({'cached': cached, 'missing': missing}))
print('Reuse:', cached, '\nBuild:', missing, flush=True)
if missing:
    subprocess.run(['colcon', '--log-base', '/tmp/atlas_map_upstream_log', 'build',
                    '--base-paths', *bases, '--packages-select', *missing,
                    '--packages-skip-build-finished', '--executor', 'sequential',
                    '--build-base', '/tmp/atlas_map_upstream_build',
                    '--install-base', '/tmp/atlas_map_upstream_install',
                    '--event-handlers', 'console_direct+', '--cmake-args',
                    '-DBUILD_TESTING=OFF', '-DCMAKE_BUILD_TYPE=Release'], check=True)
PY
source /tmp/atlas_map_upstream_install/setup.bash
colcon --log-base /tmp/atlas_map_demo_log build \
  --base-paths src/atlas_map_integration_demo \
  --build-base /tmp/atlas_map_demo_build \
  --install-base /tmp/atlas_map_demo_install \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
