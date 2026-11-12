#!/usr/bin/env python3
"""Bounded full-graph fault injection for the isolated Ackermann Nav2 fixture."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from atlas_nav2_demo.overlay_contract import is_development_package_prefix
from autoware_control_msgs.msg import Control
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import Log
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage


FAULTS = ('localization', 'obstacle_scan', 'command_loss', 'emergency_stop',
          'cancel', 'invalid_command')
HEALTHY_RECOVERY_FAULTS = {'localization', 'obstacle_scan'}
LATCHED_FAULTS = {'emergency_stop', 'invalid_command'}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1.0e-9


def json_number(value):
    return float(value) if math.isfinite(float(value)) else str(value)


def nonzero(value):
    try:
        return math.isfinite(float(value)) and abs(float(value)) > 1.0e-6
    except (TypeError, ValueError):
        return True


def clearance_to_obstacle(x, y):
    dx = max(1.75 - x, 0.0, x - 2.25)
    dy = max(-0.25 - y, 0.0, y - 0.25)
    return math.hypot(dx, dy) - 0.32


class FaultTrial(Node):
    def __init__(self, fault, output):
        super().__init__('ackermann_full_graph_fault_trial')
        self.fault = fault
        self.stream = Path(output).open('x', encoding='utf-8', buffering=1)
        self.start_mono_ns = time.monotonic_ns()
        self.deadline_ns = self.start_mono_ns + 35_000_000_000
        self.phase = 'wait_inputs'
        self.done = False
        self.failure = None
        self.gate_enabled = False
        self.goal_handle = None
        self.action_status = None
        self.cancel_requested = False
        self.fault_injected_mono_ns = None
        self.fault_injected_sim_time = None
        self.fault_injected_pose = None
        self.fault_detected_mono_ns = None
        self.first_zero_safe_mono_ns = None
        self.last_nonzero_safe_mono_ns = None
        self.stopped_sim_time = None
        self.stopped_since_mono_ns = None
        self.recovery_seen = False
        self.recovery_motion_seen = False
        self.sensor_recovery_mono_ns = None
        self.action_result_mono_ns = None
        self.cancel_requested_mono_ns = None
        self.cancel_response_mono_ns = None
        self.clear_seen = False
        self.clear_mono_ns = None
        self.rearm_seen = False
        self.stationary_after_rearm = True
        self.odom_count = 0
        self.scan_count = 0
        self.odom_receipts = []
        self.scan_receipts = []
        self.last_odom_mono_ns = None
        self.last_scan_mono_ns = None
        self.last_odom = None
        self.last_dynamics = None
        self.dynamics = []
        self.safe_samples = []
        self.gate_samples = []
        self.nav_samples = []
        self.control_samples = []
        self.controller_trace_samples = []
        self.scan_source_stamps = []
        self.plugin_stale_scan_warning_ns = None
        self.invalid_command_sent = False
        self.service_inflight = False
        self.stage_started_ns = None

        self.create_subscription(Odometry, '/atlas_ackermann/odom', self.on_odom, 50)
        self.create_subscription(LaserScan, '/atlas_ackermann/scan', self.on_scan, 10)
        self.create_subscription(String, '/atlas_ackermann/simulator_dynamics',
                                 self.on_dynamics, 50)
        self.create_subscription(Twist, '/atlas_ackermann/cmd_vel_nav',
                                 self.on_nav, 20)
        self.create_subscription(Twist, '/atlas_ackermann/cmd_vel_safe',
                                 self.on_safe, 20)
        self.create_subscription(String, '/atlas_nav2/safety_status',
                                 self.on_status, 20)
        self.create_subscription(Log, '/rosout', self.on_rosout, 100)
        self.create_subscription(Control, '/atlas_ackermann/control_cmd',
                                 self.on_control, 20)
        self.create_subscription(String,
                                 '/atlas_ackermann/controller_compute_trace',
                                 self.on_controller_trace, 30)
        self.nav_publisher = self.create_publisher(
            Twist, '/atlas_ackermann/cmd_vel_nav', 10)
        self.action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.enable_client = self.create_client(SetBool, '/safety_gate/enable')
        self.estop_client = self.create_client(
            SetBool, '/safety_gate/emergency_stop')
        self.odom_fault_client = self.create_client(
            SetBool, '/ackermann_simulator/set_odometry_enabled')
        self.scan_fault_client = self.create_client(
            SetBool, '/ackermann_synthetic_laser/set_output_enabled')
        self.controller_lifecycle_client = self.create_client(
            ChangeState, '/controller_server/change_state')
        self.timer = self.create_timer(0.02, self.tick)
        self.emit('trial_started', data={'fault': fault})

    def emit(self, event, data=None, topic=None, source_stamp=None):
        now_mono = time.monotonic_ns()
        self.stream.write(json.dumps({
            'event': event, 'topic': topic,
            'ros_now_ns': self.get_clock().now().nanoseconds,
            'mono_ns': now_mono,
            'source_stamp': source_stamp,
            'simulation_time_sec': (self.last_dynamics['simulation_time_sec']
                                    if self.last_dynamics else None),
            'data': data,
        }, allow_nan=False, separators=(',', ':')) + '\n')

    def on_odom(self, msg):
        now = time.monotonic_ns()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self.last_odom = {'x': p.x, 'y': p.y, 'yaw': yaw,
                          'speed': msg.twist.twist.linear.x,
                          'source_time': stamp_seconds(msg.header.stamp)}
        self.last_odom_mono_ns = now
        self.odom_receipts.append({'mono_ns': now})
        self.odom_count += 1
        self.emit('odometry', self.last_odom, '/atlas_ackermann/odom',
                  self.last_odom['source_time'])

    def on_scan(self, msg):
        now = time.monotonic_ns()
        self.last_scan_mono_ns = now
        self.scan_receipts.append({'mono_ns': now})
        self.scan_count += 1
        source_time = stamp_seconds(msg.header.stamp)
        self.scan_source_stamps.append((now, source_time))
        self.emit('scan', {'ranges': len(msg.ranges)},
                  '/atlas_ackermann/scan', source_time)

    def on_dynamics(self, msg):
        now = time.monotonic_ns()
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.failure = 'invalid simulator dynamics diagnostic'
            return
        data['receipt_mono_ns'] = now
        self.last_dynamics = data
        self.dynamics.append(data)
        self.emit('dynamics', data, '/atlas_ackermann/simulator_dynamics')

    def on_nav(self, msg):
        now = time.monotonic_ns()
        sample = {'mono_ns': now, 'linear': json_number(msg.linear.x),
                  'angular': json_number(msg.angular.z)}
        self.nav_samples.append(sample)
        self.emit('nav_command', sample, '/atlas_ackermann/cmd_vel_nav')

    def on_safe(self, msg):
        now = time.monotonic_ns()
        sample = {'mono_ns': now, 'linear': json_number(msg.linear.x),
                  'angular': json_number(msg.angular.z)}
        self.safe_samples.append(sample)
        if nonzero(sample['linear']) or nonzero(sample['angular']):
            self.last_nonzero_safe_mono_ns = now
        elif (self.fault_injected_mono_ns is not None and
              self.first_zero_safe_mono_ns is None):
            self.first_zero_safe_mono_ns = now
        self.emit('safe_command', sample, '/atlas_ackermann/cmd_vel_safe')

    def on_status(self, msg):
        now = time.monotonic_ns()
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            status = {'invalid_json': True}
        self.gate_samples.append((now, status))
        self.emit('gate_status', status, '/atlas_nav2/safety_status')
        if self.fault_injected_mono_ns is not None and self.fault_detected_mono_ns is None:
            if self.fault == 'localization' and not status.get('localization_fresh', True):
                self.mark_detected('gate_localization_stale')
            elif self.fault == 'command_loss' and not status.get('command_fresh', True):
                self.mark_detected('gate_command_stale')
            elif self.fault in LATCHED_FAULTS and status.get('emergency_stop', False):
                self.mark_detected('emergency_latched')

    def on_rosout(self, msg):
        now = time.monotonic_ns()
        data = {'stamp': stamp_seconds(msg.stamp), 'name': msg.name,
                'level': int(msg.level), 'message': msg.msg}
        if 'stale obstacle scan' in msg.msg.lower():
            self.plugin_stale_scan_warning_ns = now
            if self.fault == 'obstacle_scan' and self.fault_injected_mono_ns is not None:
                self.mark_detected('controller_scan_freshness_stop')
        self.emit('rosout', data, '/rosout', data['stamp'])

    def on_control(self, msg):
        now = time.monotonic_ns()
        data = {'velocity': msg.longitudinal.velocity,
                'acceleration': msg.longitudinal.acceleration,
                'steering': msg.lateral.steering_tire_angle,
                'steering_rate': msg.lateral.steering_tire_rotation_rate,
                'mono_ns': now}
        self.control_samples.append(data)
        self.emit('control', data, '/atlas_ackermann/control_cmd',
                  stamp_seconds(msg.stamp))

    def on_controller_trace(self, msg):
        now = time.monotonic_ns()
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.failure = 'invalid controller compute trace'
            return
        data['receipt_monotonic_ns'] = now
        self.controller_trace_samples.append(data)
        self.emit('controller_compute_trace', data,
                  '/atlas_ackermann/controller_compute_trace')

    def mark_detected(self, reason):
        if self.fault_detected_mono_ns is None:
            self.fault_detected_mono_ns = time.monotonic_ns()
            self.emit('fault_detected', {'reason': reason})

    def service_call(self, client, value, callback, label):
        if self.service_inflight or not client.service_is_ready():
            return False
        self.service_inflight = True
        future = client.call_async(SetBool.Request(data=value))
        future.add_done_callback(lambda result: self.on_set_bool_result(
            result, callback, label, value))
        return True

    def on_set_bool_result(self, future, callback, label, value):
        self.service_inflight = False
        try:
            response = future.result()
            self.emit('service_result', {'service': label, 'value': value,
                                         'success': response.success,
                                         'message': response.message})
            callback(bool(response.success), response.message)
        except Exception as exc:  # record the failure instead of losing evidence
            self.failure = f'{label}: {exc}'
            self.emit('service_exception', {'service': label, 'error': str(exc)})

    def send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = 4.45
        goal.pose.pose.position.y = 1.75
        goal.pose.pose.orientation.w = 1.0
        self.emit('goal_sent', {'x': 4.45, 'y': 1.75})
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        try:
            self.goal_handle = future.result()
            self.emit('goal_response', {'accepted': self.goal_handle.accepted})
            if self.goal_handle.accepted:
                self.goal_handle.get_result_async().add_done_callback(self.on_result)
            else:
                self.failure = 'Nav2 rejected fault-test goal'
        except Exception as exc:
            self.failure = f'goal response: {exc}'

    def on_result(self, future):
        try:
            result = future.result()
            self.action_status = int(result.status)
            self.action_result_mono_ns = time.monotonic_ns()
            name = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(
                self.action_status, 'UNKNOWN')
            self.emit('action_result', {'status': self.action_status,
                                        'status_name': name})
            if self.fault == 'cancel' and self.action_status == 5:
                self.mark_detected('navigation_action_canceled')
        except Exception as exc:
            self.emit('action_result_exception', {'error': str(exc)})

    def inject_fault(self):
        self.fault_injected_mono_ns = time.monotonic_ns()
        self.fault_injected_sim_time = (self.last_dynamics['simulation_time_sec']
                                        if self.last_dynamics else None)
        if self.last_dynamics:
            self.fault_injected_pose = {
                'x': self.last_dynamics['x_m'], 'y': self.last_dynamics['y_m'],
                'speed': self.last_dynamics['speed_after_mps']}
        self.phase = 'fault_active'
        self.stage_started_ns = self.fault_injected_mono_ns
        self.emit('fault_injected', {
            'fault': self.fault,
            'pose': self.fault_injected_pose,
            'speed_mps': (self.fault_injected_pose or {}).get('speed'),
        })
        if self.fault == 'localization':
            self.service_call(self.odom_fault_client, False, self.on_fault_service,
                              'pause_odometry')
        elif self.fault == 'obstacle_scan':
            self.service_call(self.scan_fault_client, False, self.on_fault_service,
                              'pause_scan')
        elif self.fault == 'command_loss':
            self.request_controller_deactivate()
        elif self.fault == 'emergency_stop':
            self.service_call(self.estop_client, True, self.on_fault_service,
                              'emergency_stop')
        elif self.fault == 'cancel':
            self.cancel_goal('injected_navigation_cancel')
        elif self.fault == 'invalid_command':
            self.invalid_command_sent = True
            command = Twist()
            command.linear.x = float('nan')
            self.nav_publisher.publish(command)
            self.emit('invalid_command_injected', {'linear_x': 'NaN'})

    def on_fault_service(self, success, message):
        if not success:
            self.failure = f'fault service rejected: {message}'
        else:
            self.emit('fault_applied', {'fault': self.fault, 'message': message})

    def request_controller_deactivate(self):
        if not self.controller_lifecycle_client.service_is_ready():
            self.failure = 'controller lifecycle service unavailable'
            return
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self.controller_lifecycle_client.call_async(request)
        future.add_done_callback(self.on_lifecycle_result)

    def on_lifecycle_result(self, future):
        try:
            response = future.result()
            self.emit('controller_deactivate', {'success': response.success})
            if response.success:
                self.emit('fault_applied', {'fault': self.fault,
                                            'reason': 'controller deactivated'})
            else:
                self.failure = 'controller server refused deactivation'
        except Exception as exc:
            self.failure = f'controller deactivate: {exc}'

    def cancel_goal(self, reason):
        if self.goal_handle is None or self.cancel_requested:
            return
        self.cancel_requested = True
        self.cancel_requested_mono_ns = time.monotonic_ns()
        self.emit('cancel_requested', {'reason': reason})
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.on_cancel_response)

    def on_cancel_response(self, future):
        try:
            response = future.result()
            self.cancel_response_mono_ns = time.monotonic_ns()
            self.emit('cancel_response', {
                'goals_canceling': len(response.goals_canceling)})
        except Exception as exc:
            self.failure = f'cancel request: {exc}'

    def begin_sensor_recovery(self):
        self.phase = 'recover_sensor'
        self.stage_started_ns = time.monotonic_ns()
        if self.fault == 'localization':
            self.service_call(self.odom_fault_client, True, self.on_recovery_service,
                              'restore_odometry')
        else:
            self.service_call(self.scan_fault_client, True, self.on_recovery_service,
                              'restore_scan')

    def on_recovery_service(self, success, message):
        if not success:
            self.failure = f'sensor recovery failed: {message}'
        else:
            self.sensor_recovery_mono_ns = time.monotonic_ns()
            self.emit('sensor_recovery_requested', {'message': message})

    def begin_latched_rearm(self):
        self.phase = 'latched_cancel'
        self.stage_started_ns = time.monotonic_ns()
        self.cancel_goal('prevent_resume_before_explicit_rearm')

    def begin_clear_latch(self):
        self.phase = 'clear_latch'
        self.stage_started_ns = time.monotonic_ns()
        self.service_call(self.estop_client, False, self.on_clear_result,
                          'clear_emergency_stop')

    def on_clear_result(self, success, message):
        self.clear_seen = success
        self.clear_mono_ns = time.monotonic_ns()
        self.emit('emergency_clear', {'success': success, 'message': message})
        if not success:
            self.failure = 'emergency stop clear failed'

    def begin_explicit_rearm(self):
        self.phase = 'explicit_rearm'
        self.stage_started_ns = time.monotonic_ns()
        self.service_call(self.enable_client, True, self.on_rearm_result,
                          'explicit_gate_enable')

    def on_rearm_result(self, success, message):
        self.rearm_seen = success
        self.emit('explicit_rearm', {'success': success, 'message': message})
        if not success:
            self.failure = 'explicit gate rearm failed after E-stop clear'

    def tick(self):
        now = time.monotonic_ns()
        if self.done:
            return
        if self.failure:
            self.finish(False, self.failure)
            return
        if now > self.deadline_ns:
            self.finish(False, 'fault trial timed out in phase ' + self.phase)
            return
        if self.phase == 'wait_inputs':
            if (self.odom_count >= 3 and self.scan_count >= 2 and
                    self.action_client.server_is_ready() and
                    self.enable_client.service_is_ready()):
                self.phase = 'enable_gate'
                self.service_call(self.enable_client, True, self.on_gate_enabled,
                                  'enable_gate')
        elif self.phase == 'wait_motion':
            fresh_odom = (self.last_odom_mono_ns is not None and
                          now - self.last_odom_mono_ns < 300_000_000)
            fresh_scan = (self.last_scan_mono_ns is not None and
                          now - self.last_scan_mono_ns < 300_000_000)
            moving_safe = any(abs(s['linear']) > 0.05 for s in self.safe_samples[-8:])
            moving = bool(self.last_dynamics and
                          abs(self.last_dynamics['speed_after_mps']) >= 0.08)
            if fresh_odom and fresh_scan and moving_safe and moving:
                self.inject_fault()
        elif self.phase == 'fault_active':
            if self.fault_detected_mono_ns is not None:
                if self.first_zero_safe_mono_ns is not None and self.is_stopped():
                    if self.fault in HEALTHY_RECOVERY_FAULTS:
                        self.begin_sensor_recovery()
                    elif self.fault in LATCHED_FAULTS:
                        self.begin_latched_rearm()
                    else:
                        self.finish_fault_response()
            if now - self.stage_started_ns > 5_000_000_000:
                self.finish(False, 'fault did not produce detected safe output')
        elif self.phase == 'recover_sensor':
            self.update_recovery_seen()
            if (self.recovery_seen and self.last_dynamics and
                    abs(self.last_dynamics['speed_after_mps']) >= 0.05):
                self.recovery_motion_seen = True
                self.cancel_goal('stop_after_verified_sensor_recovery')
                self.phase = 'recovery_cancel'
                self.stage_started_ns = now
            elif now - self.stage_started_ns > 4_000_000_000:
                self.finish(False, 'sensor did not recover to healthy navigation')
        elif self.phase == 'recovery_cancel':
            if self.action_status == 5 and self.is_stopped():
                self.finish(True, 'sensor recovered, resumed, then canceled safely')
            elif now - self.stage_started_ns > 6_000_000_000:
                self.finish(False, 'recovered navigation failed to cancel and stop')
        elif self.phase == 'latched_cancel':
            if self.action_status in (5, 6) and self.is_stopped():
                self.begin_clear_latch()
            elif now - self.stage_started_ns > 4_000_000_000:
                self.finish(False, 'E-stop navigation action did not terminate safely')
        elif self.phase == 'clear_latch':
            if self.clear_seen and self.stopped_hold(300_000_000):
                # Verify clearing the E-stop alone does not enable the gate.
                if self.latest_status().get('enabled', True):
                    self.finish(False, 'emergency clear automatically re-enabled gate')
                else:
                    self.begin_explicit_rearm()
            elif now - self.stage_started_ns > 3_000_000_000:
                self.finish(False, 'gate did not remain stopped after E-stop clear')
        elif self.phase == 'explicit_rearm':
            self.stationary_after_rearm = self.stationary_after_rearm and self.is_stopped()
            if self.rearm_seen and self.stopped_hold(500_000_000):
                if not self.stationary_after_rearm or self.action_status not in (5, 6):
                    self.finish(False, 'vehicle resumed old goal after explicit rearm')
                else:
                    self.finish(True, 'E-stop latched; clear stayed disabled; explicit rearm did not resume old goal')
            elif now - self.stage_started_ns > 3_000_000_000:
                self.finish(False, 'explicit rearm verification timed out')
        elif self.phase == 'finish_response':
            if self.is_stopped() and self.action_status in (4, 5, 6):
                self.finish(True, 'fault response reached safe standstill')

    def on_gate_enabled(self, success, message):
        if not success:
            self.failure = f'gate enable failed: {message}'
            return
        self.gate_enabled = True
        self.phase = 'send_goal'
        self.send_goal()
        self.phase = 'wait_motion'

    def latest_status(self):
        return self.gate_samples[-1][1] if self.gate_samples else {}

    def update_recovery_seen(self):
        if self.fault == 'localization':
            self.recovery_seen = bool(self.latest_status().get('localization_fresh'))
        else:
            now = time.monotonic_ns()
            fresh_scan = (self.last_scan_mono_ns is not None and
                          now - self.last_scan_mono_ns < 300_000_000)
            self.recovery_seen = fresh_scan and self.plugin_stale_scan_warning_ns is not None

    def is_stopped(self):
        return bool(self.last_dynamics and
                    abs(self.last_dynamics['speed_after_mps']) <= 0.005)

    def stopped_hold(self, hold_ns):
        now = time.monotonic_ns()
        if not self.is_stopped():
            self.stopped_since_mono_ns = None
            return False
        if self.stopped_since_mono_ns is None:
            self.stopped_since_mono_ns = now
        return now - self.stopped_since_mono_ns >= hold_ns

    def finish_fault_response(self):
        self.phase = 'finish_response'
        self.stage_started_ns = time.monotonic_ns()
        if self.goal_handle and self.action_status is None:
            self.cancel_goal('fault_test_cleanup')

    def finish(self, passed, message):
        if self.done:
            return
        metrics = self.metrics()
        if passed:
            passed, checks = self.validate_metrics(metrics)
            metrics['acceptance_checks'] = checks
            if not passed:
                message = 'fault response failed a measured safety assertion'
        self.done = True
        result = {'fault': self.fault, 'passed': bool(passed),
                  'message': message, 'metrics': metrics}
        self.emit('trial_result', result)
        self.stream.flush()
        self.stream.close()

    def metrics(self):
        injection = self.fault_injected_mono_ns
        dynamics = self.dynamics
        if injection is None:
            return {'failure': self.failure, 'action_status': self.action_status}
        before = [d for d in dynamics
                  if d['receipt_mono_ns'] <= injection]
        after = [d for d in dynamics
                 if d['receipt_mono_ns'] >= injection]
        start = before[-1] if before else (dynamics[0] if dynamics else None)
        stopped = next((d for d in after
                        if abs(d['speed_after_mps']) <= 0.005), None)
        stop_distance = None
        stop_sim_duration = None
        min_clearance = None
        acceleration_max = None
        deceleration_min = None
        steering_rate_max = None
        if start and stopped:
            segment = [d for d in dynamics
                       if start['simulation_time_sec'] <= d['simulation_time_sec'] <=
                       stopped['simulation_time_sec']]
            stop_distance = sum(math.hypot(b['x_m'] - a['x_m'],
                                           b['y_m'] - a['y_m'])
                                for a, b in zip(segment, segment[1:]))
            stop_sim_duration = (stopped['simulation_time_sec'] -
                                 start['simulation_time_sec'])
        if dynamics:
            min_clearance = min(clearance_to_obstacle(d['x_m'], d['y_m'])
                                for d in dynamics)
            acceleration_max = max(d['acceleration_mps2'] for d in dynamics)
            deceleration_min = min(d['acceleration_mps2'] for d in dynamics)
            steering_rate_max = max(abs(d['steering_rate_radps']) for d in dynamics)
        zero_delay = ((self.first_zero_safe_mono_ns - injection) * 1e-9
                      if self.first_zero_safe_mono_ns and injection else None)
        detection_delay = ((self.fault_detected_mono_ns - injection) * 1e-9
                           if self.fault_detected_mono_ns and injection else None)
        peak_speed_before_zero = None
        if self.first_zero_safe_mono_ns:
            before_zero = [d for d in dynamics
                           if injection <= d['receipt_mono_ns'] <=
                           self.first_zero_safe_mono_ns]
            # Include the final simulator sample preceding injection. An
            # immediate E-stop can zero the gate before another 20 ms dynamics
            # sample arrives, but the pre-injection sample is synchronized on
            # the simulator's own timebase and is the conservative speed input.
            candidates = [abs(d['speed_after_mps']) for d in before_zero]
            if start:
                candidates.append(abs(start['speed_after_mps']))
            if candidates:
                peak_speed_before_zero = max(candidates)
        stopping_envelope = None
        if peak_speed_before_zero is not None and zero_delay is not None:
            stopping_envelope = (peak_speed_before_zero * zero_delay +
                                 peak_speed_before_zero ** 2 / (2.0 * 1.20) +
                                 0.05)
        safe_zero_until = (self.sensor_recovery_mono_ns
                           if self.fault in HEALTHY_RECOVERY_FAULTS else
                           self.action_result_mono_ns or time.monotonic_ns())
        no_nonzero_after_stop_command = bool(
            self.first_zero_safe_mono_ns and
            not any(s['mono_ns'] >= self.first_zero_safe_mono_ns and
                    s['mono_ns'] <= safe_zero_until and
                    (nonzero(s['linear']) or nonzero(s['angular']))
                    for s in self.safe_samples))
        cancel_origin = (self.cancel_requested_mono_ns
                         if self.fault == 'cancel' else injection)
        cancel_action_result_latency = (
            (self.action_result_mono_ns - cancel_origin) * 1e-9
            if self.fault == 'cancel' and self.action_result_mono_ns else None)
        cancel_ack_latency = (
            (self.cancel_response_mono_ns - cancel_origin) * 1e-9
            if self.fault == 'cancel' and self.cancel_response_mono_ns else None)
        cancel_safe_zero_latency = (
            (self.first_zero_safe_mono_ns - cancel_origin) * 1e-9
            if self.fault == 'cancel' and self.first_zero_safe_mono_ns else None)

        status_zero_publish = next((
            status.get('safe_output_publish_monotonic_ns')
            for mono, status in self.gate_samples
            if self.fault == 'cancel' and mono >= cancel_origin and
            int(status.get('safe_output_publish_monotonic_ns') or 0) >= cancel_origin and
            abs(status.get('output_linear', math.inf)) <= 1.0e-6 and
            abs(status.get('output_angular', math.inf)) <= 1.0e-6), None)
        cancel_gate_zero_latency = (
            (status_zero_publish - cancel_origin) * 1e-9
            if status_zero_publish else None)
        control_zero = next((sample['mono_ns'] for sample in self.control_samples
                             if self.fault == 'cancel' and
                             sample['mono_ns'] >= cancel_origin and
                             abs(sample['velocity']) <= 1.0e-6), None)
        cancel_control_zero_latency = (
            (control_zero - cancel_origin) * 1e-9 if control_zero else None)
        sim_zero = next((
            d['tick_monotonic_ns'] for d in dynamics
            if self.fault == 'cancel' and
            d.get('tick_monotonic_ns', 0) >= cancel_origin and
            d.get('control_receipt_monotonic_ns') is not None and
            d['control_receipt_monotonic_ns'] >= cancel_origin and
            abs(d.get('target_speed_mps', math.inf)) <= 1.0e-6), None)
        cancel_sim_zero_latency = (
            (sim_zero - cancel_origin) * 1e-9 if sim_zero else None)
        cancel_standstill_latency = (
            (stopped['receipt_mono_ns'] - cancel_origin) * 1e-9
            if self.fault == 'cancel' and stopped and
            stopped.get('receipt_mono_ns') else None)

        controller_trace_delivery_lag = [
            (sample['receipt_monotonic_ns'] - sample['callback_end_monotonic_ns']) * 1e-9
            for sample in self.controller_trace_samples
            if 'callback_end_monotonic_ns' in sample]
        controller_callback_durations = [
            sample.get('callback_duration_ns', 0) * 1e-9
            for sample in self.controller_trace_samples]
        status_delivery_lags = [
            (mono - int(status['safe_output_publish_monotonic_ns'])) * 1e-9
            for mono, status in self.gate_samples
            if status.get('safe_output_publish_monotonic_ns') is not None]
        dynamics_delivery_lags = [
            (d['receipt_mono_ns'] - d['tick_monotonic_ns']) * 1e-9
            for d in dynamics if d.get('tick_monotonic_ns') is not None]
        first_zero_reference = self.first_zero_safe_mono_ns or time.monotonic_ns()
        last_nonzero_controller = max((s['mono_ns'] for s in self.nav_samples
                                       if s['mono_ns'] <= first_zero_reference and
                                       (nonzero(s['linear']) or nonzero(s['angular']))),
                                      default=None)
        last_nonzero_gate_output = max((s['mono_ns'] for s in self.safe_samples
                                        if s['mono_ns'] <= first_zero_reference and
                                        (nonzero(s['linear']) or nonzero(s['angular']))),
                                       default=None)
        last_nonzero_adapter_control = max((s['mono_ns'] for s in self.control_samples
                                            if s['mono_ns'] <= first_zero_reference and
                                            nonzero(s['velocity'])), default=None)
        last_nonzero_sim_target = max((d['tick_monotonic_ns'] for d in dynamics
                                      if d.get('tick_monotonic_ns', 0) <=
                                      first_zero_reference and
                                      nonzero(d.get('target_speed_mps', 0.0))),
                                     default=None)
        # Report delivery/scheduling gaps alongside stop-envelope measurements.
        # A paused simulator cannot accumulate simulated travel, but the gap is
        # still relevant to command freshness and must not be hidden by that.
        def max_gap(receipts):
            stamps = sorted(t for t in receipts if injection <= t)
            if self.first_zero_safe_mono_ns is not None:
                stamps = [t for t in stamps if t <= self.first_zero_safe_mono_ns]
            return max((b - a) * 1e-9 for a, b in zip(stamps, stamps[1:])) \
                if len(stamps) > 1 else None

        gate_receipts = [t for t, _ in self.gate_samples]
        dynamics_receipts = [d['receipt_mono_ns'] for d in dynamics]
        odom_receipts = [self.last_odom_mono_ns] if self.last_odom_mono_ns else []
        # Keep all odometry receipt times separately: last_odom_mono_ns is only
        # the latest sample and is insufficient for a gap measurement.
        odom_receipts = [int(row['mono_ns']) for row in self.odom_receipts]
        scan_receipts = [int(row['mono_ns']) for row in self.scan_receipts]
        gate_gap = max_gap(gate_receipts)
        dynamics_gap = max_gap(dynamics_receipts)
        odom_gap = max_gap(odom_receipts)
        scan_gap = max_gap(scan_receipts)
        return {
            'fault_injected_pose': self.fault_injected_pose,
            'fault_speed_mps': (self.fault_injected_pose or {}).get('speed'),
            'fault_detection_latency_monotonic_s': detection_delay,
            'first_zero_safe_command_latency_monotonic_s': zero_delay,
            'cancel_to_cancel_ack_monotonic_s': cancel_ack_latency,
            'cancel_to_action_result_monotonic_s': cancel_action_result_latency,
            'cancel_to_first_safe_zero_receipt_monotonic_s': cancel_safe_zero_latency,
            'cancel_to_gate_internal_zero_publish_monotonic_s':
                cancel_gate_zero_latency,
            'cancel_to_adapter_zero_control_publish_monotonic_s':
                cancel_control_zero_latency,
            'cancel_to_simulator_zero_target_applied_monotonic_s':
                cancel_sim_zero_latency,
            'cancel_to_simulated_standstill_monotonic_s':
                cancel_standstill_latency,
            'last_nonzero_controller_command_before_gate_zero_monotonic_ns':
                last_nonzero_controller,
            'last_nonzero_gate_output_before_gate_zero_monotonic_ns':
                last_nonzero_gate_output,
            'last_nonzero_adapter_control_before_gate_zero_monotonic_ns':
                last_nonzero_adapter_control,
            'last_nonzero_simulator_target_before_gate_zero_monotonic_ns':
                last_nonzero_sim_target,
            'controller_compute_max_callback_duration_s':
                max(controller_callback_durations, default=None),
            'controller_compute_max_trace_delivery_lag_s':
                max(controller_trace_delivery_lag, default=None),
            'gate_status_max_internal_to_recorder_delivery_lag_s':
                max(status_delivery_lags, default=None),
            'simulator_max_tick_to_recorder_delivery_lag_s':
                max(dynamics_delivery_lags, default=None),
            'max_gate_status_interarrival_before_safe_zero_s': gate_gap,
            'max_simulator_dynamics_interarrival_before_safe_zero_s': dynamics_gap,
            'max_odometry_interarrival_before_safe_zero_s': odom_gap,
            'max_laserscan_interarrival_before_safe_zero_s': scan_gap,
            'last_nonzero_safe_command_mono_ns': self.last_nonzero_safe_mono_ns,
            'stop_duration_simulation_s': stop_sim_duration,
            'post_fault_travel_distance_m': stop_distance,
            'peak_speed_before_first_zero_mps': peak_speed_before_zero,
            'conservative_stopping_envelope_m': stopping_envelope,
            'stopping_envelope_respected': (
                stop_distance is not None and stopping_envelope is not None and
                stop_distance <= stopping_envelope + 1.0e-6),
            'no_nonzero_safe_output_after_stop_until_recovery_or_action_result':
                no_nonzero_after_stop_command,
            'minimum_obstacle_clearance_m': min_clearance,
            'max_step_acceleration_mps2': acceleration_max,
            'min_step_acceleration_mps2': deceleration_min,
            'max_step_steering_rate_radps': steering_rate_max,
            'action_status': self.action_status,
            'safe_samples_after_fault': sum(
                s['mono_ns'] >= injection for s in self.safe_samples),
            'nonzero_safe_after_fault': any(
                s['mono_ns'] >= injection and
                (nonzero(s['linear']) or nonzero(s['angular']))
                for s in self.safe_samples),
            'emergency_latched': any(s.get('emergency_stop', False)
                                     for _, s in self.gate_samples),
            'clear_without_rearm_verified': self.clear_seen,
            'clear_mono_ns': self.clear_mono_ns,
            'explicit_rearm_verified': self.rearm_seen,
            'sensor_recovery_verified': self.recovery_seen,
            'resumed_motion_verified': self.recovery_motion_seen,
            'odom_samples': self.odom_count,
            'scan_samples': self.scan_count,
        }

    def validate_metrics(self, metrics):
        checks = {
            'injected_while_moving': metrics.get('fault_speed_mps', 0.0) >= 0.08,
            'safe_zero_observed': metrics.get(
                'first_zero_safe_command_latency_monotonic_s') is not None,
            'stopped': metrics.get('stop_duration_simulation_s') is not None,
            'stopping_envelope_respected': metrics.get(
                'stopping_envelope_respected') is True,
            'no_collision_or_footprint_overlap': (
                metrics.get('minimum_obstacle_clearance_m') is not None and
                metrics['minimum_obstacle_clearance_m'] > 0.0),
            'no_nonzero_after_stop_command': metrics.get(
                'no_nonzero_safe_output_after_stop_until_recovery_or_action_result') is True,
            'simulator_acceleration_bounded': (
                metrics.get('max_step_acceleration_mps2', math.inf) <= 0.80000001 and
                metrics.get('min_step_acceleration_mps2', -math.inf) >= -1.20000001),
        }
        if self.fault == 'cancel':
            checks['action_canceled'] = self.action_status == 5
            checks['safe_zero_within_command_freshness_window'] = (
                metrics.get('first_zero_safe_command_latency_monotonic_s',
                            math.inf) <= 0.25)
        elif self.fault == 'localization':
            checks['localization_stale_detected'] = (
                self.fault_detected_mono_ns is not None)
            checks['healthy_localization_recovered'] = self.recovery_seen
            checks['navigation_resumed_after_recovery'] = self.recovery_motion_seen
            checks['recovered_goal_canceled_safely'] = self.action_status == 5
        elif self.fault == 'obstacle_scan':
            checks['controller_detected_stale_scan'] = (
                self.plugin_stale_scan_warning_ns is not None)
            checks['healthy_scan_recovered'] = self.recovery_seen
            checks['navigation_resumed_after_recovery'] = self.recovery_motion_seen
            checks['recovered_goal_canceled_safely'] = self.action_status == 5
        elif self.fault == 'command_loss':
            checks['gate_detected_command_loss'] = (
                self.fault_detected_mono_ns is not None and
                any(not s.get('command_fresh', True) for _, s in self.gate_samples))
            checks['navigation_terminated'] = self.action_status in (5, 6)
        else:
            checks['emergency_latched'] = metrics.get('emergency_latched') is True
            checks['clear_did_not_rearm'] = metrics.get(
                'clear_without_rearm_verified') is True
            checks['explicit_rearm_observed'] = metrics.get(
                'explicit_rearm_verified') is True
            checks['old_goal_did_not_resume'] = (
                self.action_status in (5, 6) and self.stationary_after_rearm)
            checks['gate_disabled_after_latch'] = all(
                not status.get('enabled', True)
                for mono, status in self.gate_samples
                if mono >= (self.fault_detected_mono_ns or self.start_mono_ns) and
                (self.clear_mono_ns is None or mono < self.clear_mono_ns))
        return all(checks.values()), checks


def stop_process(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def wait_launch(process, log_path, timeout=45.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f'Nav2 launch exited {process.returncode}')
        if 'Managed nodes are active' in log_path.read_text(
                encoding='utf-8', errors='replace'):
            return
        time.sleep(0.20)
    raise TimeoutError('Nav2 lifecycle did not become active')


def run_trial(fault, domain, evidence_root, share):
    stem = f'{fault}_domain{domain}'
    capture = evidence_root / f'{stem}.jsonl'
    launch_log_path = evidence_root / f'{stem}_launch.log'
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain)
    launch_command = ['ros2', 'launch', 'atlas_nav2_demo',
                      'nav2_ackermann_controller.launch.py',
                      'scenario:=avoidable',
                      f'map:={share / "maps" / "avoidable_map.yaml"}']
    with launch_log_path.open('x', encoding='utf-8') as launch_log:
        launch = subprocess.Popen(launch_command, env=env, stdout=launch_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        trial = None
        try:
            wait_launch(launch, launch_log_path)
            os.environ['ROS_DOMAIN_ID'] = str(domain)
            rclpy.init()
            trial = FaultTrial(fault, capture)
            trial_deadline = time.monotonic() + 40.0
            while rclpy.ok() and not trial.done and time.monotonic() < trial_deadline:
                rclpy.spin_once(trial, timeout_sec=0.05)
            if not trial.done:
                trial.finish(False, 'orchestrator wall timeout')
            result = json.loads(capture.read_text(encoding='utf-8').splitlines()[-1])['data']
            trial.destroy_node()
            rclpy.try_shutdown()
        except Exception as exc:
            result = {'fault': fault, 'passed': False, 'message': str(exc)}
            if trial is not None:
                try:
                    trial.stream.close()
                    trial.destroy_node()
                except Exception:
                    pass
            if rclpy.ok():
                rclpy.try_shutdown()
        finally:
            stop_process(launch)
    result['domain'] = domain
    result['capture'] = str(capture)
    result['launch_log'] = str(launch_log_path)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fault', choices=(*FAULTS, 'all'), default='all')
    parser.add_argument('--domain-start', type=int, default=231)
    parser.add_argument('--evidence-root', type=Path, required=True)
    args = parser.parse_args(argv)
    faults = FAULTS if args.fault == 'all' else (args.fault,)
    if args.domain_start < 0 or args.domain_start + len(faults) - 1 > 232:
        parser.error('Fast DDS test domains must be within 0..232')
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    prefix = Path(get_package_prefix('atlas_nav2_demo')).resolve()
    if not is_development_package_prefix(prefix):
        parser.error(
            'development package must resolve from the first COLCON_PREFIX_PATH '
            'entry (or ATLAS_NAV2_OVERLAY); resolved package prefix: '
            f'{prefix}')
    share = Path(get_package_share_directory('atlas_nav2_demo'))
    args.evidence_root.mkdir(parents=True, exist_ok=True)
    results = []
    for index, fault in enumerate(faults):
        result = run_trial(fault, args.domain_start + index,
                           args.evidence_root, share)
        results.append(result)
        print(json.dumps(result, indent=2), flush=True)
        if not result.get('passed'):
            break
    report = {'package_prefix': str(prefix), 'requested_faults': list(faults),
              'results': results, 'passed': len(results) == len(faults) and
              all(result.get('passed') for result in results)}
    (args.evidence_root / 'fault_summary.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
