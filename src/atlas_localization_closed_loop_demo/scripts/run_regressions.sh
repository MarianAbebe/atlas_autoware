#!/usr/bin/env bash
# Existing standalone NDT and original ground-truth-fed driving tests, serially.
set -eo pipefail
atlas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$atlas_root"
regression_output="${1:?Provide a fresh regression output directory}"
mkdir "$regression_output"
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
source .localization_cache/install/setup.bash
source .perception_cache/install/local_setup.bash
source .perception_cache/atlas_install/local_setup.bash
export ENABLE_AGNOCAST=0 CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
python3 src/atlas_closed_loop_demo/scripts/colcon_sync_io.py --log-base .localization_cache/ndt_regression_log build \
  --paths src/atlas_ndt_localization_test --build-base .localization_cache/ndt_regression_build \
  --install-base .localization_cache/ndt_regression_install --executor sequential \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  > "$regression_output/ndt_build.log" 2>&1
source .localization_cache/ndt_regression_install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
set +e
ROS_DOMAIN_ID=84 ctest --test-dir .localization_cache/ndt_regression_build/atlas_ndt_localization_test \
  --output-on-failure > "$regression_output/ndt_ctest.log" 2>&1
ndt_status=$?
bash src/atlas_closed_loop_demo/scripts/run_demo.sh --no-rviz --gated \
  --timeout 125 --results-root "$regression_output/closed_loop_nominal" \
  > "$regression_output/closed_loop_nominal.log" 2>&1
nominal_status=$?
python3 src/atlas_closed_loop_demo/scripts/run_safety_suite.py \
  --results-root "$regression_output/closed_loop_faults" \
  > "$regression_output/closed_loop_faults.log" 2>&1
fault_status=$?
python3 - "$regression_output" "$ndt_status" "$nominal_status" "$fault_status" <<'PY'
import json,sys
from pathlib import Path
summary=dict(zip(['ndt','closed_loop_nominal','closed_loop_faults'],map(int,sys.argv[2:])))
(Path(sys.argv[1])/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary))
raise SystemExit(0 if not any(summary.values()) else 1)
PY
