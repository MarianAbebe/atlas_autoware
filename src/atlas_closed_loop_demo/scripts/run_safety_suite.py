#!/usr/bin/env python3
"""Run the six bounded simulation-only interruption cases sequentially."""
import json
import argparse
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parents[3]
ap=argparse.ArgumentParser();ap.add_argument('--results-root',type=Path,default=Path(__file__).resolve().parents[1]/'results/safety_validation/faults_final');args=ap.parse_args()
results=args.results_root
results.mkdir(parents=True,exist_ok=True)
summary=[]
for channel in ['odometry','steering','command']:
    for fault in ['drop','freeze']:
        log=results/(channel+'_'+fault+'.log')
        command=['bash','src/atlas_closed_loop_demo/scripts/run_demo.sh','--no-rviz','--gated',
                 '--vehicle-model','IDEAL_STEER_ACC_GEARED','--fault',fault,'--fault-input',channel,
                 '--timeout','45','--results-root',str(results)]
        with log.open('w') as output:
            result=subprocess.run(command,cwd=root,stdout=output,stderr=subprocess.STDOUT,timeout=90)
        row={'input':channel,'fault':fault,'exit_code':result.returncode,'log':str(log)}
        summary.append(row)
        (results/'suite.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps(row),flush=True)
sys.exit(0 if all(r['exit_code']==0 for r in summary) else 1)
