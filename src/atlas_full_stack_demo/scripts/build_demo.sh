#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
source src/atlas_full_stack_demo/scripts/environment.sh
python3 - <<'PINS'
from pathlib import Path
import subprocess
import os
reference=Path(os.environ['ATLAS_AUTOWARE_ROOT'])
for suffix,sha in [('', '10718787ba6e28f038a0cb29ff99cc627b5abfd2'), ('src/core/autoware_core','f25f83c632c1984ec276c894c41857d4abc0dad8'),('src/universe/autoware_universe','6e477c645efec33f7909095eea684474e97f5e3d'),('src/core/autoware_simple_planning_simulator','82d2c93a28a25f8802e8f5a21b4ec850beea9080')]:
 path=reference/suffix
 assert subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()==sha,str(path)
 assert not subprocess.check_output(['git','-C',str(path),'status','--porcelain','--untracked-files=no'],text=True),str(path)
PINS
export ENABLE_AGNOCAST=0 CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
cmake -S "$ATLAS_AUTOWARE_ROOT/src/core/autoware_core/perception/autoware_euclidean_cluster_object_detector" -B .full_stack_cache/cluster_build -DCMAKE_INSTALL_PREFIX="$PWD/.full_stack_cache/cluster_install" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build .full_stack_cache/cluster_build -j1
cmake --install .full_stack_cache/cluster_build
python3 src/atlas_closed_loop_demo/scripts/colcon_sync_io.py --log-base .localization_cache/demo_log build --paths src/atlas_localization_closed_loop_demo --build-base .localization_cache/demo_build --install-base .localization_cache/demo_install --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
python3 src/atlas_closed_loop_demo/scripts/colcon_sync_io.py --log-base .full_stack_cache/log build --paths src/atlas_perception_planning_demo src/atlas_full_stack_demo --build-base .full_stack_cache/build --install-base .full_stack_cache/install --executor sequential --cmake-args -DBUILD_TESTING=OFF

python3 src/atlas_full_stack_demo/scripts/test_static_objects.py
python3 src/atlas_full_stack_demo/scripts/test_scheduler_telemetry.py
