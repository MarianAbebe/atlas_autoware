#!/usr/bin/env python3
"""Sequential bounded tests on isolated ROS domain 80; no manual controls."""
import argparse
import json
from pathlib import Path
import subprocess
import time


def main():
    root = Path(__file__).resolve().parents[3]
    demo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--skip-none', action='store_true', help='Use only when a separate fresh no-obstacle run is recorded')
    parser.add_argument('--results-root', type=Path, default=demo/'results'/time.strftime('suite_%Y%m%d_%H%M%S'))
    args = parser.parse_args()
    results = args.results_root.resolve()
    results.mkdir(parents=True, exist_ok=True)
    tests = [] if args.skip_none else [('none', 'odometry')]
    tests += [(s, 'odometry') for s in ['ahead', 'off_route', 'stale_drop', 'stale_freeze']]
    tests += [(s, c) for c in ['odometry', 'steering', 'command'] for s in ['feedback_drop', 'feedback_freeze']]
    summary = []
    for scenario, channel in tests:
        log = results/(time.strftime('%Y%m%d_%H%M%S')+'_'+scenario+'_'+channel+'.log')
        command = ['bash', str(demo/'scripts/run_test.sh'), '--scenario', scenario,
                   '--feedback-input', channel, '--results-root', str(results)]
        with log.open('w') as output:
            result = subprocess.run(command, cwd=root, stdout=output, stderr=subprocess.STDOUT, timeout=210)
        row = {'scenario': scenario, 'feedback_input': channel, 'exit_code': result.returncode, 'log': str(log)}
        summary.append(row)
        (results/'suite.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(row), flush=True)
    if not all(row['exit_code'] == 0 for row in summary):
        return 1
    if not args.skip_none:
        subprocess.run(['python3', str(demo/'scripts/review_results.py'), '--results-root', str(results)],
                       cwd=root, check=True, timeout=60)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
