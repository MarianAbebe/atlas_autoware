#!/usr/bin/env python3
"""Fail early when this environment cannot run the isolated ROS/RViz test."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-rviz', action='store_true',default=True)
    parser.add_argument('--rviz',dest='no_rviz',action='store_false')
    args, _ = parser.parse_known_args()
    report = {'checks': {}, 'errors': {}, 'display': os.environ.get('DISPLAY')}
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1', 0))
        report['checks']['local_udp_socket'] = True
    except OSError as exc:
        report['checks']['local_udp_socket'] = False
        report['errors']['local_udp_socket'] = str(exc)
    if not args.no_rviz:
        try:
            result = subprocess.run(['xset', '-q'], capture_output=True, text=True, timeout=3)
            report['checks']['x11_display'] = result.returncode == 0
            if result.returncode:
                report['errors']['x11_display'] = result.stderr.strip()
        except (OSError, subprocess.TimeoutExpired) as exc:
            report['checks']['x11_display'] = False
            report['errors']['x11_display'] = str(exc)
    probe_dir = Path(__file__).resolve().parents[1] / 'results/safety_validation/dds_preflight'
    probe = subprocess.run([__import__('sys').executable, str(Path(__file__).with_name('dds_probe.py')), '--results', str(probe_dir)], capture_output=True, text=True, timeout=30)
    report['checks']['dds_message_delivery'] = probe.returncode == 0
    report['udp_socket_is_diagnostic_only'] = True
    # A denied socket is diagnostic evidence, not proof that DDS delivery fails.
    required = ['dds_message_delivery'] + ([] if args.no_rviz else ['x11_display'])
    report['passed'] = all(report['checks'][k] for k in required)
    report['scope'] = 'Environment probe only; no simulation or vehicle commands executed.'
    target = Path(__file__).resolve().parents[1] / 'results/current_audit/runtime_preflight.json'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
