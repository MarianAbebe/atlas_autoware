#!/usr/bin/env python3
"""Bounded live safety-gate validation; no Nav2, simulator, or motor interface."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class GateProbe(Node):
    def __init__(self):
        super().__init__('atlas_nav2_gate_probe')
        self.outputs = []
        self.statuses = []
        self.command = self.create_publisher(Twist, '/atlas_nav2/cmd_vel_nav', 10)
        self.localization = self.create_publisher(Odometry, '/atlas_nav2/odom', 20)
        self.create_subscription(Twist, '/atlas_nav2/cmd_vel_safe', self.on_output, 20)
        self.create_subscription(String, '/atlas_nav2/safety_status', self.on_status, 20)
        self.enable = self.create_client(SetBool, '/safety_gate/enable')
        self.emergency = self.create_client(SetBool, '/safety_gate/emergency_stop')

    def on_output(self, message):
        self.outputs.append((time.monotonic(), message.linear.x, message.angular.z))

    def on_status(self, message):
        try:
            self.statuses.append((time.monotonic(), json.loads(message.data)))
        except ValueError:
            self.statuses.append((time.monotonic(), {'invalid_json': True}))

    def publish_localization(self, invalid=False):
        message = Odometry()
        if invalid:
            message.header.stamp.sec = -1
        else:
            message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.orientation.w = 1.0
        self.localization.publish(message)

    def publish_command(self):
        message = Twist()
        message.linear.x = 0.20
        message.angular.z = 0.15
        self.command.publish(message)


def spin_for(node, seconds, localization=False, command=False, invalid=False):
    deadline = time.monotonic() + seconds
    next_publish = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_publish:
            if localization:
                node.publish_localization(invalid=invalid)
            if command:
                node.publish_command()
            next_publish = now + 0.04
        rclpy.spin_once(node, timeout_sec=0.02)


def call(node, client, value):
    deadline = time.monotonic() + 5.0
    while not client.service_is_ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not client.service_is_ready():
        raise RuntimeError(f'service unavailable: {client.srv_name}')
    future = client.call_async(SetBool.Request(data=value))
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done() or not future.result().success:
        raise RuntimeError(f'service failed: {client.srv_name}')


def all_zero(samples):
    return bool(samples) and all(abs(linear) <= 1e-9 and abs(angular) <= 1e-9
                                 for _, linear, angular in samples)


def stop_group(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {'result': 'RUNNING', 'checks': {}, 'purpose':
              'isolated synthetic safety-gate validation; no physical motor topics'}
    process = None
    node = None
    log = (output / 'safety_gate.log').open('w')
    try:
        process = subprocess.Popen(['ros2', 'run', 'atlas_nav2_demo', 'safety_gate'],
                                   stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=os.environ.copy())
        rclpy.init()
        node = GateProbe()

        deadline = time.monotonic() + 5.0
        while not node.statuses and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.statuses:
            raise RuntimeError('safety gate did not publish status')
        node.outputs.clear()
        spin_for(node, 0.35, command=True)
        report['checks']['missing_localization_zero'] = (
            all_zero(node.outputs[-5:])
            and not node.statuses[-1][1].get('localization_fresh', True))
        call(node, node.enable, True)

        start = len(node.outputs)
        spin_for(node, 0.40, localization=True, command=True)
        report['checks']['fresh_command_and_localization_forwarded'] = any(
            abs(linear) > 0.02 or abs(angular) > 0.02
            for _, linear, angular in node.outputs[start:])

        start = len(node.outputs)
        spin_for(node, 0.40, localization=True)
        # The first part of this window may legitimately still be inside the
        # configured timeout. Inspect only the terminal gate samples.
        report['checks']['stale_command_zero'] = (
            all_zero(node.outputs[start:][-5:])
            and not node.statuses[-1][1].get('command_fresh', True))

        start = len(node.outputs)
        spin_for(node, 0.40, command=True)
        report['checks']['stale_localization_zero'] = (
            all_zero(node.outputs[start:][-5:])
            and not node.statuses[-1][1].get('localization_fresh', True))

        start = len(node.outputs)
        spin_for(node, 0.40, command=True, localization=True, invalid=True)
        report['checks']['invalid_timestamp_zero'] = all_zero(node.outputs[start:][-5:])

        spin_for(node, 0.30, localization=True, command=True)
        call(node, node.emergency, True)
        start = len(node.outputs)
        spin_for(node, 0.20, localization=True, command=True)
        report['checks']['emergency_stop_zero'] = all_zero(node.outputs[start:])
        call(node, node.emergency, False)
        start = len(node.outputs)
        spin_for(node, 0.20, localization=True, command=True)
        report['checks']['emergency_clear_does_not_reenable'] = all_zero(node.outputs[start:])
        report['checks']['valid_status_json'] = bool(node.statuses) and not any(
            'invalid_json' in status for _, status in node.statuses)
        if not all(report['checks'].values()):
            raise RuntimeError('one or more safety-gate assertions failed')
        report['result'] = 'PASS'
    except Exception as exc:
        report['result'] = 'FAIL'
        report['failure'] = str(exc)
    finally:
        if node is not None:
            report['final_status'] = node.statuses[-1][1] if node.statuses else None
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        if process is not None:
            stop_group(process)
        log.close()
        (output / 'summary.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
