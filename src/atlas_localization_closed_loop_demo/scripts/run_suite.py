#!/usr/bin/env python3
"""Run only the isolated localization scenarios, serially and with fresh evidence."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    script = Path(__file__).with_name('run_test.sh')
    summary = []
    for scenario in ['nominal','drop_ndt','drop_ekf','no_ndt','no_ekf']:
        with (args.output/(scenario+'.log')).open('w') as log:
            result = subprocess.run(['bash',str(script),'--scenario',scenario,'--output',
                str(args.output/scenario)],stdout=log,stderr=subprocess.STDOUT,timeout=90)
        summary.append(dict(scenario=scenario,exit_code=result.returncode))
        (args.output/'suite.json').write_text(json.dumps(summary,indent=2))
        print(summary[-1],flush=True)
    return 0 if all(r['exit_code']==0 for r in summary) else 1


if __name__ == '__main__':
    raise SystemExit(main())
