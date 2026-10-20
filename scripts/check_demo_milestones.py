#!/usr/bin/env python3
"""Fast source/configuration and historical evidence checks; no ROS commands."""
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    names = ['closed_loop_demo', 'lidar_perception_test', 'map_integration_demo',
             'trajectory_generation_demo', 'vehicle_motion_demo']
    counts = {'python': 0, 'shell': 0, 'xml': 0, 'yaml': 0}
    for name in names:
        package = root / 'src' / ('atlas_' + name)
        for path in package.rglob('*'):
            if 'results' in path.relative_to(package).parts or not path.is_file():
                continue
            if path.suffix == '.py':
                compile(path.read_text(), str(path), 'exec')
                counts['python'] += 1
            elif path.suffix == '.sh':
                subprocess.run(['bash', '-n', str(path)], check=True)
                counts['shell'] += 1
            elif path.suffix == '.xml':
                ET.parse(path)
                counts['xml'] += 1
            elif path.suffix in ('.yaml', '.rviz'):
                yaml.safe_load(path.read_text())
                counts['yaml'] += 1
    evidence = root / 'src/atlas_closed_loop_demo/results/safety_validation'
    summary = json.loads((evidence / 'release_summary.json').read_text())
    limits = json.loads((evidence / 'thresholds.json').read_text())
    assert summary['fresh_closed_loop_passed'] and summary['six_fault_cases_passed']
    assert summary['rendered_image_inspection'] == 'unverified'
    m = summary['normal_metrics']
    for metric, limit in [('final_distance_to_goal_m', 'goal_distance_max_m'),
                          ('max_lateral_error_m', 'lateral_max_m'),
                          ('rms_lateral_error_m', 'lateral_rms_max_m'),
                          ('max_heading_error_rad', 'heading_max_rad'),
                          ('max_actual_speed_mps', 'speed_max_mps'),
                          ('terminal_hold_creep_m', 'terminal_creep_distance_max_m')]:
        assert m[metric] <= limits[limit], metric
    assert m['terminal_hold_sec'] >= limits['terminal_hold_sec']
    assert m['final_speed_mps'] == 0
    cases = summary['fault_cases']
    assert {(c['input'], c['fault']) for c in cases} == {
        (channel, fault) for channel in ('odometry', 'steering', 'command')
        for fault in ('drop', 'freeze')}
    for case in cases:
        assert case['passed'] and case['automatic_rearm_prevented']
        assert case['invalid_rearm_refused'] and case['fresh_inputs_restored']
        assert case['final_speed_mps'] == case['post_stop_motion_m'] == 0
        assert case['detection_sec'] <= limits['fault_detection_latency_max_sec']
        assert case['stopping_distance_m'] <= limits['fault_stop_distance_max_m']
        assert case['settling_sec'] <= limits['fault_stop_settling_max_sec']
    manifest = json.loads((evidence / 'evidence_manifest.json').read_text())['files']
    checked = 0
    for relative, entry in manifest.items():
        path = evidence.parents[1] / relative
        if path.exists():
            assert path.stat().st_size == entry['bytes'], relative
            assert hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256'], relative
            checked += 1
    print('PASS:', counts, '; historical summary/threshold checks passed; local evidence',
          f'{checked}/{len(manifest)} files verified (absent recordings are not rerun).')


if __name__ == '__main__':
    main()
