#!/usr/bin/env bash
# Build missing pinned localization packages only; never install dependencies.
set -eo pipefail
atlas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$atlas_root"
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
source .closed_loop_cache/demo_install/setup.bash
export ENABLE_AGNOCAST=0 CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
if [[ -f .localization_cache/install/setup.bash ]]; then source .localization_cache/install/setup.bash; fi
python3 - <<'PY'
from pathlib import Path
import json, os, subprocess
from catkin_pkg.package import parse_package
root=Path.cwd();ref=Path(os.environ.get('ATLAS_AUTOWARE_ROOT', root.parent/'references/autoware'))
for path,sha in [(ref,'10718787ba6e28f038a0cb29ff99cc627b5abfd2'),
 (ref/'src/core/autoware_core','f25f83c632c1984ec276c894c41857d4abc0dad8'),
 (ref/'src/universe/autoware_universe','6e477c645efec33f7909095eea684474e97f5e3d'),
 (ref/'src/core/autoware_simple_planning_simulator','82d2c93a28a25f8802e8f5a21b4ec850beea9080')]:
 assert subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()==sha, str(path)
prefixes=os.environ['AMENT_PREFIX_PATH'].split(':')
allp={}
for path in (ref/'src/core').rglob('package.xml'):
 p=parse_package(path);allp[p.name]=(p,path.parent)
missing={}
def visit(name):
 if any(Path(p,'share',name,'package.xml').exists() for p in prefixes) or name in missing:return
 if name not in allp:raise RuntimeError('Missing prerequisite: '+name)
 p,path=allp[name];missing[name]=str(path);p.evaluate_conditions(dict(os.environ))
 for key in ['build_depends','buildtool_depends','build_export_depends','buildtool_export_depends','exec_depends']:
  for edge in getattr(p,key):
   if edge.evaluated_condition and edge.name in allp:visit(edge.name)
for name in ['autoware_ndt_scan_matcher','autoware_ekf_localizer']:visit(name)
cache=root/'.localization_cache';cache.mkdir(exist_ok=True)
(cache/'build_selection.json').write_text(json.dumps(missing,indent=2))
if missing:
 subprocess.run(['rosdep','check','--rosdistro','humble','--ignore-src',
  *['--dependency-types='+t for t in ['build','buildtool','build_export','buildtool_export','exec']],
  '--from-paths',*missing.values()],check=True)
 subprocess.run(['python3','src/atlas_closed_loop_demo/scripts/colcon_sync_io.py',
  '--log-base',str(cache/'log'),'build','--paths',*missing.values(),
  '--build-base',str(cache/'build'),'--install-base',str(cache/'install'),
  '--executor','sequential','--event-handlers','console_direct+','--cmake-args',
  '-DBUILD_TESTING=OFF','-DCMAKE_BUILD_TYPE=Release',
  '-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=gold','-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=gold'],check=True)
PY
source .localization_cache/install/setup.bash
python3 src/atlas_closed_loop_demo/scripts/colcon_sync_io.py --log-base .localization_cache/demo_log build \
  --paths src/atlas_localization_closed_loop_demo --build-base .localization_cache/demo_build \
  --install-base .localization_cache/demo_install --executor sequential --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
