#!/usr/bin/env python3
"""Bounded two-process DDS test; publishes only an isolated std_msgs/String."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid


def worker(role, token, target):
    import rclpy
    from std_msgs.msg import String
    result = {'role': role, 'received': [], 'published': 0}
    rclpy.init()
    node = rclpy.create_node('atlas_dds_probe_' + role)
    topic = '/atlas_closed_loop_environment_probe/' + token
    if role == 'subscriber':
        node.create_subscription(String, topic, lambda m: result['received'].append(m.data), 10)
    else:
        pub = node.create_publisher(String, topic, 10)
    start = time.monotonic()
    while time.monotonic() - start < (7 if role == 'subscriber' else 6):
        if role == 'publisher':
            msg = String(); msg.data = token + ':' + str(result['published'])
            pub.publish(msg); result['published'] += 1
        rclpy.spin_once(node, timeout_sec=.05)
    node.destroy_node(); rclpy.shutdown()
    target.write_text(json.dumps(result, indent=2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--role', choices=['publisher', 'subscriber'])
    ap.add_argument('--token')
    ap.add_argument('--output', type=Path)
    ap.add_argument('--results', type=Path, default=Path(__file__).resolve().parents[1]/'results/safety_validation/dds_probe')
    args = ap.parse_args()
    if args.role:
        worker(args.role, args.token, args.output); return 0
    args.results.mkdir(parents=True, exist_ok=True)
    result = {'uid': os.getuid(), 'middleware': 'rmw_fastrtps_cpp', 'domain': 79,
              'scope': 'Environment test only; no simulator or vehicle commands.'}
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(('127.0.0.1', 0))
        result['udp_socket'] = 'permitted'
    except OSError as exc:
        result['udp_socket'] = str(exc)
    result['restrictions'] = [l for l in Path('/proc/self/status').read_text().splitlines()
                              if l.startswith(('NoNewPrivs:', 'Seccomp:', 'CapEff:'))]
    result['original_ros_environment'] = {k: v for k, v in os.environ.items()
                                         if k.startswith(('ROS_', 'RMW_', 'FAST', 'CYCLONE'))}
    env = dict(os.environ, ROS_DOMAIN_ID='79', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               RCUTILS_COLORIZED_OUTPUT='0')
    token = 'probe_' + uuid.uuid4().hex
    jobs = []
    for role in ['subscriber', 'publisher']:
        target = args.results/(token+'_'+role+'.json')
        log = (args.results/(token+'_'+role+'.log')).open('w')
        proc = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--role', role,
                                 '--token', token, '--output', str(target)], env=env,
                                stdout=log, stderr=subprocess.STDOUT)
        jobs.append((role, proc, target, log))
    for role, proc, target, log in jobs:
        try: code = proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try: code = proc.wait(timeout=2)
            except subprocess.TimeoutExpired: proc.kill(); code = proc.wait()
        log.close()
        result[role] = {'exit_code': code, 'result': json.loads(target.read_text()) if target.exists() else None}
    subscriber = result['subscriber']['result']
    result['passed'] = bool(subscriber and len(set(subscriber['received'])) >= 3
                            and all(r.startswith(token+':') for r in subscriber['received'])
                            and all(result[r]['exit_code'] == 0 for r in ['publisher', 'subscriber']))
    (args.results/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
