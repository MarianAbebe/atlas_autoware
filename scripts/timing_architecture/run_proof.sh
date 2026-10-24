#!/usr/bin/env bash
# Isolated timing proof only; no driving suites and no system installations.
set -eo pipefail
atlas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$atlas_root"
proof_output="${1:?Provide a fresh output directory}"
mkdir "$proof_output"
proof_output="$(realpath "$proof_output")"
source /opt/ros/humble/setup.bash
source .closed_loop_cache/install/setup.bash
source .closed_loop_cache/demo_install/setup.bash
source .localization_cache/install/setup.bash
source .localization_cache/demo_install/setup.bash
export ROS_DOMAIN_ID=85 PYTHONDONTWRITEBYTECODE=1
python3 - <<'PY'
from pathlib import Path
import subprocess
ref=Path.home()/'roam/references/autoware'
for path,sha in [(ref,'10718787ba6e28f038a0cb29ff99cc627b5abfd2'),
 (ref/'src/core/autoware_simple_planning_simulator','82d2c93a28a25f8802e8f5a21b4ec850beea9080')]:
 assert subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()==sha,str(path)
PY
cmake -S scripts/timing_architecture -B .localization_cache/timing_build \
  -DCMAKE_BUILD_TYPE=Release > "$proof_output/build.log" 2>&1
cmake --build .localization_cache/timing_build -j1 >> "$proof_output/build.log" 2>&1
python3 scripts/timing_architecture/audit_runtime.py --output "$proof_output/runtime" > "$proof_output/runtime.log" 2>&1
.localization_cache/timing_build/pinned_step_probe realtime 500 > "$proof_output/realtime_500.log" 2>&1
.localization_cache/timing_build/pinned_step_probe controlled 500 > "$proof_output/controlled_500.log" 2>&1
.localization_cache/timing_build/pinned_step_probe controlled 0 > "$proof_output/controlled_0.log" 2>&1
.localization_cache/timing_build/polling_clock_probe > "$proof_output/polling_clock.log" 2>&1
python3 scripts/timing_architecture/test_clock_contracts.py > "$proof_output/clock_tests.log" 2>&1
python3 scripts/test_command_observation.py > "$proof_output/observation_before.log" 2>&1
python3 scripts/timing_architecture/test_observation_barrier.py > "$proof_output/observation_after.log" 2>&1
python3 scripts/timing_architecture/test_command_journal.py > "$proof_output/command_journal.log" 2>&1
python3 - "$proof_output" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]);runs={}
for n in ['realtime_500','controlled_500','controlled_0']:
 for line in (p/(n+'.log')).read_text().splitlines():
  if line.startswith('ATLAS_TIMING_RESULT '):runs[n]=json.loads(line.split(' ',1)[1])
assert len(runs)==3
for key in ['final_x','final_vx','samples']:
 assert abs(runs['controlled_500'][key]-runs['controlled_0'][key])<1e-12,key
polling=next(json.loads(line.split(' ',1)[1]) for line in (p/'polling_clock.log').read_text().splitlines() if line.startswith('ATLAS_POLLING_RESULT '))
assert polling['mismatch_reproduced']
result=dict(scope='component timing proof only',full_loop_corrected=False,historical_root_cause_established=False,
 runs=runs,controlled_final_state_identical=True,polling_clock=polling,passed=True)
(p/'step_results.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
PY
