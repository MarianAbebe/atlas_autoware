#!/usr/bin/env python3
"""Offline consistency audit of all eleven recorded cases; never publishes ROS."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def main():
    demo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-root', type=Path, default=demo/'results/final')
    args = parser.parse_args()
    reports = sorted(args.results_root.glob('*/validation.json'))
    expected = {(s, 'odometry') for s in ['none', 'ahead', 'off_route', 'stale_drop', 'stale_freeze']}
    expected |= {(s, c) for s in ['feedback_drop', 'feedback_freeze'] for c in ['odometry', 'steering', 'command']}
    seen, reviewed = set(), []
    reference = None
    limits = json.loads((demo/'config/thresholds.json').read_text())
    for report in reports:
        value = json.loads(report.read_text())
        m = value['metrics']
        key = m['scenario'], m['feedback_input']
        assert key not in seen, f'Duplicate case {key}; use a fresh results-root for each suite'
        seen.add(key)
        assert value['passed'] and not value['failed_checks'] and all(value['checks'].values()), report
        folder = report.parent
        assert json.loads((folder/'thresholds.json').read_text()) == limits, report
        recorded = json.loads((folder/'source_sha256.json').read_text())
        live = {k: v for k, v in recorded.items() if k.startswith(('launch/', 'config/'))
                or k in ['scripts/obstacle_source.py', 'scripts/obstacle_guard.py']}
        assert live == {k: hashlib.sha256((demo/k).read_bytes()).hexdigest() for k in live}, report
        if reference is None:
            reference = live
        assert live == reference, report
        graph = json.loads((folder/'graph.json').read_text())
        for topic, publisher in [('obstacles/pointcloud', 'obstacle_source'),
                                 ('obstacles/predicted_objects', 'obstacle_source'),
                                 ('obstacle_trajectory', 'motion_velocity_planner'),
                                 ('trajectory', 'velocity_smoother'),
                                 ('controller_raw/control_cmd', 'controller')]:
            assert graph['/atlas_closed_loop/'+topic]['publishers'] == ['/atlas_closed_loop/'+publisher], topic
        a = np.genfromtxt(folder/'odometry.csv', delimiter=',', names=True)
        distance = float(np.hypot(np.diff(a['x']), np.diff(a['y'])).sum())
        assert abs(distance-m['total_distance_m']) < 1e-8, report
        row = {'recording': str(folder), 'raw_distance_recomputed_m': distance,
               'passed_checks': len(value['checks']), 'final_speed_mps': float(a['speed'][-1])}
        if m['scenario'] in ['none', 'off_route']:
            trajectories = json.loads((folder/'trajectories.json').read_text())
            last = a[-1]
            count = sum(t['kind'] == 'modified' and any(abs(p[3]) < 1e-5 and
                math.hypot(p[0]-last['x'], p[1]-last['y']) > 5 for p in t['points']) for t in trajectories)
            assert count == 0, report
            row['nonterminal_stop_insertions'] = count
        log = (folder/'runtime.log').read_text()
        measurement, cleanup = log.split('ATLAS_MEASUREMENT_COMPLETE', 1)
        unclassified = [line for line in measurement.splitlines() if '[ERROR]' in line and '[VehicleCmdFilter]' not in line]
        assert not unclassified, unclassified
        assert 'SIGKILL' not in cleanup, report
        row['gate_filter_events'] = measurement.count('[VehicleCmdFilter]')
        row['osqp_polish_warnings'] = measurement.count('Polish process failed')
        row['shutdown_sigterm_escalations'] = sum('escalating to' in line and 'SIGTERM' in line for line in cleanup.splitlines())
        row['sha256'] = {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                         for f in folder.iterdir() if f.is_file() and f.suffix in ['.csv', '.json']}
        reviewed.append(row)
    assert seen == expected, f'Missing or unexpected cases: {expected ^ seen}'
    output = {'passed': True, 'live_configuration_identical': True, 'thresholds_identical': True,
              'scope': 'Offline review of recorded evidence; no new motion execution', 'recordings': reviewed}
    (args.results_root/'review_audit.json').write_text(json.dumps(output, indent=2))
    print(f'PASS: {len(reviewed)} recordings, {sum(r["passed_checks"] for r in reviewed)} recorded checks; raw distance, source/configuration, graph, thresholds and log consistency verified.')


if __name__ == '__main__':
    main()
