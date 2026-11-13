#!/usr/bin/env python3
"""Bounded full-graph Autoware gate E-stop test for the simulated Atlas route."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import Engage
from tier4_external_api_msgs.msg import Emergency
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Capture(Node):
    def __init__(self):
        super().__init__('atlas_full_stack_estop_validation')
        self.odom = None
        self.last_position = None
        self.distance_after_fault = 0.0
        self.fault_mono = None
        self.speed_at_fault = None
        self.first_zero_command_mono = None
        self.first_emergency_report_mono = None
        self.first_speed_drop_mono = None
        self.stopped_since = None
        self.gate_emergency = False
        self.gate_engaged = None
        self.last_gate = None
        self.last_controller = None
        self.status = {}
        self.rows = []
        self.create_subscription(Odometry, '/atlas_loc/sim/output/odometry', self.on_odom, 20)
        self.create_subscription(Control, '/atlas_loc/gated/control_cmd', self.on_gate, 20)
        self.create_subscription(Control, '/atlas_loc/control_cmd', self.on_controller, 20)
        self.create_subscription(Emergency,
            '/atlas_loc/gate/output/external_emergency', self.on_emergency, 10)
        self.create_subscription(Engage, '/atlas_loc/gate/output/engage', self.on_engage, 10)
        self.create_subscription(String, '/atlas_loc/safety_status', self.on_status, 10)
        self.create_subscription(String, '/atlas_loc/base_safety_status', self.on_status, 10)
        self.clear_mono = None
        self.resumed_motion_command = False
        self.clear_estop = self.create_client(Trigger,
            '/atlas_loc/gate/vehicle_cmd_gate/service/clear_external_emergency_stop')
        self.estop = self.create_client(Trigger,
            '/atlas_loc/gate/vehicle_cmd_gate/service/external_emergency_stop')

    def on_odom(self, msg):
        mono = time.monotonic()
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if self.fault_mono is not None and self.last_position is not None:
            self.distance_after_fault += math.dist(xy, self.last_position)
            if speed < 0.02:
                if self.stopped_since is None:
                    self.stopped_since = mono
            else:
                self.stopped_since = None
            if self.speed_at_fault is not None and speed < self.speed_at_fault - 0.01 and self.first_speed_drop_mono is None:
                self.first_speed_drop_mono = mono
        self.last_position = xy
        self.odom = {'mono': mono, 'ros_ns': msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                     'xy': xy, 'speed': speed}
        self.record('odometry')

    def on_gate(self, msg):
        self.last_gate = msg
        now = time.monotonic()
        if self.fault_mono is not None and msg.longitudinal.velocity <= 0.001 and self.first_zero_command_mono is None:
            self.first_zero_command_mono = now
        if self.clear_mono is not None and now >= self.clear_mono and msg.longitudinal.velocity > 0.02:
            self.resumed_motion_command = True
        self.record('gated_command')

    def on_controller(self, msg):
        self.last_controller = msg

    def on_emergency(self, msg):
        self.gate_emergency = bool(msg.emergency)
        if self.fault_mono is not None and self.gate_emergency and self.first_emergency_report_mono is None:
            self.first_emergency_report_mono = time.monotonic()
        self.record('gate_emergency')

    def on_engage(self, msg):
        self.gate_engaged = bool(msg.engage)

    def on_status(self, msg):
        try:
            self.status[msg._topic_name if hasattr(msg, '_topic_name') else 'latest'] = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def record(self, event):
        if self.odom is None:
            return
        gate = self.last_gate
        cmd = self.last_controller
        self.rows.append({
            'event': event, 'receipt_monotonic_sec': time.monotonic(),
            'odom_ros_stamp_ns': self.odom['ros_ns'], 'speed_mps': self.odom['speed'],
            'x': self.odom['xy'][0], 'y': self.odom['xy'][1],
            'gate_velocity_mps': None if gate is None else gate.longitudinal.velocity,
            'gate_acceleration_mps2': None if gate is None else gate.longitudinal.acceleration,
            'gate_steering_rad': None if gate is None else gate.lateral.steering_tire_angle,
            'controller_velocity_mps': None if cmd is None else cmd.longitudinal.velocity,
            'gate_emergency': self.gate_emergency,
        })


def spin_until(node, predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


def stop_group(proc):
    if proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=5)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--laps', type=int, choices=(1, 2, 5), default=1)
    ap.add_argument('--timeout', type=float, default=180.0)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'metadata.json').write_text(json.dumps({
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'), 'laps': args.laps,
        'freshness_sec': 0.3, 'simulator_deceleration_limit_mps2': 1.2,
        'gate_estop_acceleration_mps2': -0.5,
        'note': 'Gate emergency command is observed at simulator input; no physical actuator is connected.'
    }, indent=2) + '\n')
    log = (args.output / 'runtime.log').open('w')
    proc = None
    rclpy.init()
    node = Capture()
    result = {'passed': False, 'status': 'not_started'}
    try:
        command = ['ros2', 'launch', 'atlas_full_stack_demo', 'five_lap_demo.launch.py',
                   f'laps:={args.laps}', f'results_dir:={args.output}',
                   'launch_rviz:=false', 'launch_ego_marker:=false']
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        start = time.monotonic()
        ready = spin_until(node, lambda: (node.odom is not None and node.odom['speed'] >= 0.2
                           and node.last_gate is not None and node.last_gate.longitudinal.velocity > 0.1
                           and node.estop.service_is_ready() and node.clear_estop.service_is_ready()), min(args.timeout, 90.0))
        if not ready:
            result = {'status': 'failed_to_reach_healthy_motion', 'passed': False}
        else:
            node.fault_mono = time.monotonic()
            node.speed_at_fault = node.odom['speed']
            response_future = node.estop.call_async(Trigger.Request())
            if not spin_until(node, response_future.done, 5.0):
                result = {'status': 'estop_service_timeout', 'passed': False}
            else:
                response = response_future.result()
                service_ok = bool(response and response.success)
                stopped = spin_until(node, lambda: (node.gate_emergency and node.first_zero_command_mono is not None
                                  and node.stopped_since is not None and node.gate_engaged is False
                                  and time.monotonic() - node.stopped_since >= 1.0), args.timeout)
                # E-stop clearing is tested only after standstill. If the old
                # controller command becomes authorized again, re-latch immediately.
                latch_held_before_clear = node.gate_emergency
                clear_future = node.clear_estop.call_async(Trigger.Request())
                clear_completed = spin_until(node, clear_future.done, 5.0)
                clear_response = clear_future.result() if clear_completed else None
                clear_accepted = bool(clear_response and clear_response.success)
                node.clear_mono = time.monotonic()
                clear_observed = spin_until(node, lambda: node.resumed_motion_command, 1.0)
                if clear_observed:
                    relatch = node.estop.call_async(Trigger.Request())
                    spin_until(node, relatch.done, 3.0)
                    spin_until(node, lambda: node.gate_emergency, 2.0)
                    spin_until(node, lambda: node.odom is not None and node.odom['speed'] < 0.02, 8.0)
                # The gate must stay latched until explicit clear; the old route
                # command must remain inhibited after that explicit clear.
                latch_held = latch_held_before_clear
                elapsed = time.monotonic() - node.fault_mono
                v0 = node.speed_at_fault or 0.0
                # Gate emergency acceleration is -0.5 m/s^2; include 300 ms
                # detection/command latency and 0.10 m modeling margin.
                envelope = v0 * 0.3 + v0 * v0 / (2.0 * 0.5) + 0.10
                result = {
                    'status': 'estop_stopped_latched' if stopped and service_ok and latch_held and not clear_observed else 'estop_failed_or_auto_resumed',
                    'passed': bool(stopped and service_ok and latch_held
                                   and node.distance_after_fault <= envelope
                                   and node.first_speed_drop_mono is not None
                                   and latch_held_before_clear and clear_accepted
                                   and not clear_observed),
                    'service_accepted': service_ok,
                    'fault_injection_monotonic_sec': node.fault_mono,
                    'fault_injection_ros_stamp_ns': None if node.odom is None else node.odom['ros_ns'],
                    'speed_at_injection_mps': v0,
                    'first_zero_gate_command_latency_sec': None if node.first_zero_command_mono is None else node.first_zero_command_mono-node.fault_mono,
                    'gate_emergency_report_latency_sec': None if node.first_emergency_report_mono is None else node.first_emergency_report_mono-node.fault_mono,
                    'first_simulated_deceleration_latency_sec': None if node.first_speed_drop_mono is None else node.first_speed_drop_mono-node.fault_mono,
                    'standstill_time_sec': elapsed,
                    'distance_after_fault_m': node.distance_after_fault,
                    'allowed_simulation_envelope_m': envelope,
                    'gate_emergency_latched_until_explicit_clear': latch_held,
                    'explicit_clear_accepted': clear_accepted,
                    'old_route_command_resumed_after_clear': clear_observed,
                    'controller_command_still_present': node.last_controller is not None,
                    'final_controller_velocity_mps': None if node.last_controller is None else node.last_controller.longitudinal.velocity,
                    'gate_output_velocity_mps': None if node.last_gate is None else node.last_gate.longitudinal.velocity,
                    'final_speed_mps': None if node.odom is None else node.odom['speed'],
                    'gate_engaged_after_estop': node.gate_engaged,
                    'safety_status': node.status,
                    'limitation': 'This is simulation-only software-path evidence; it is not hardware E-stop validation.'
                }
    except Exception as exc:
        result = {'status': 'exception', 'passed': False, 'error': f'{type(exc).__name__}: {exc}'}
    finally:
        if proc is not None:
            stop_group(proc)
        log.close()
        (args.output / 'samples.jsonl').write_text(''.join(json.dumps(x, allow_nan=False) + '\n' for x in node.rows))
        (args.output / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
