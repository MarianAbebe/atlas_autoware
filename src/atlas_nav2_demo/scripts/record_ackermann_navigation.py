#!/usr/bin/env python3
"""Bounded JSONL capture of one isolated Ackermann Nav2 goal and its inputs."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from autoware_control_msgs.msg import Control
from geometry_msgs.msg import PolygonStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from rcl_interfaces.msg import Log
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage


def stamp_dict(stamp):
    return {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}


def finite(value):
    value = float(value)
    return value if math.isfinite(value) else None


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AckermannRunRecorder(Node):
    def __init__(self, args):
        super().__init__('ackermann_run_recorder')
        self.args = args
        self.started_mono_ns = time.monotonic_ns()
        self.started_ros_ns = self.get_clock().now().nanoseconds
        self.file = Path(args.output)
        self.file.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.file.open('x', encoding='utf-8', buffering=1)
        self.done = False
        self.finish_requested = False
        self.finish_deadline_ns = None
        self.settling = False
        self.settle_deadline_ns = None
        self.stopped_since_ns = None
        self.post_action_deadline_ns = None
        self.last_odom = None
        self.last_odom_receive_ns = None
        self.last_scan_receive_ns = None
        self.gate_was_enabled = False
        self.goal_handle = None
        self.action_status = None
        self.last_feedback_mono_ns = 0
        self.feedback_period_ns = 100_000_000  # Keep feedback at or below 10 Hz.
        self.deadline_ns = self.started_mono_ns + int(args.timeout * 1e9)

        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.enable_client = self.create_client(SetBool, '/safety_gate/enable')
        self.create_subscription(NavPath, '/plan', self.on_path, 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(Twist, '/atlas_ackermann/cmd_vel_nav',
                                 self.on_nav_cmd, 20)
        self.create_subscription(Twist, '/atlas_ackermann/cmd_vel_safe',
                                 self.on_safe_cmd, 20)
        self.create_subscription(String, '/atlas_ackermann/command_status',
                                 self.on_adapter_status, 20)
        self.create_subscription(Control, '/atlas_ackermann/control_cmd',
                                 self.on_control, 20)
        self.create_subscription(String, '/atlas_ackermann/simulator_dynamics',
                                 self.on_simulator_dynamics, 50)
        self.create_subscription(String, '/atlas_nav2/safety_status',
                                 self.on_gate_status, 20)
        self.create_subscription(Odometry, '/atlas_ackermann/odom',
                                 self.on_odom, 50)
        self.create_subscription(LaserScan, '/atlas_ackermann/scan',
                                 self.on_scan, 10)
        tf_qos = QoSProfile(depth=100,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        tf_static_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf', self.on_tf, tf_qos)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf_static,
                                 tf_static_qos)
        self.create_subscription(Log, '/rosout', self.on_rosout,
                                 QoSProfile(depth=100,
                                            reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Costmap, '/local_costmap/costmap',
                                 self.on_local_costmap, 5)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                 self.on_local_costmap_raw, 5)
        self.create_subscription(Costmap, '/global_costmap/costmap',
                                 self.on_global_costmap, 5)
        self.create_subscription(Costmap, '/global_costmap/costmap_raw',
                                 self.on_global_costmap_raw, 5)
        self.create_subscription(PolygonStamped,
                                 '/local_costmap/published_footprint',
                                 self.on_local_footprint, 5)
        self.create_subscription(PolygonStamped,
                                 '/global_costmap/published_footprint',
                                 self.on_global_footprint, 5)
        self.timer = self.create_timer(0.05, self.poll)
        self.emit('capture_started', data={
            'goal_frame': args.frame,
            'goal_x': args.goal_x,
            'goal_y': args.goal_y,
            'goal_yaw': args.goal_yaw,
            'timeout_sec': args.timeout,
            'action_feedback_max_hz': 10,
            'gate_enable_requested': args.enable_gate,
        })

    def emit(self, event, topic=None, header=None, data=None,
             receive_mono_ns=None, callback_start_ns=None):
        now_mono = time.monotonic_ns()
        record = {
            'event': event,
            'topic': topic,
            'ros_now_ns': self.get_clock().now().nanoseconds,
            'mono_receive_ns': receive_mono_ns or now_mono,
            'callback_duration_ns': (now_mono - callback_start_ns
                                     if callback_start_ns else 0),
            'header_stamp': stamp_dict(header.stamp) if header else None,
            'frame_id': header.frame_id if header else None,
            'data': data,
        }
        self.stream.write(json.dumps(record, separators=(',', ':'),
                                      allow_nan=False) + '\n')

    def poll(self):
        now = time.monotonic_ns()
        if self.settling:
            if self.last_odom is not None:
                pose = self.last_odom
                xy_error = math.hypot(pose['x'] - self.args.goal_x,
                                      pose['y'] - self.args.goal_y)
                yaw_error = abs(math.atan2(
                    math.sin(pose['yaw'] - self.args.goal_yaw),
                    math.cos(pose['yaw'] - self.args.goal_yaw)))
                stopped = abs(pose['linear_x']) <= self.args.stopped_speed
                in_goal = (xy_error <= self.args.goal_xy_tolerance and
                           yaw_error <= self.args.goal_yaw_tolerance)
                if stopped and in_goal:
                    if self.stopped_since_ns is None:
                        self.stopped_since_ns = now
                    elif now - self.stopped_since_ns >= 300_000_000:
                        self.emit('settled_result', data={
                            'passed': True, 'xy_error_m': xy_error,
                            'yaw_error_rad': yaw_error,
                            'speed_mps': pose['linear_x']})
                        self.settling = False
                        self.finish(0)
                        return
                else:
                    self.stopped_since_ns = None
            if now >= self.settle_deadline_ns:
                pose = self.last_odom
                if pose is None:
                    assessment = {'passed': False, 'reason': 'missing_odometry'}
                else:
                    assessment = {
                        'passed': False, 'reason': 'not_stopped_or_outside_goal',
                        'xy_error_m': math.hypot(
                            pose['x'] - self.args.goal_x,
                            pose['y'] - self.args.goal_y),
                        'yaw_error_rad': abs(math.atan2(
                            math.sin(pose['yaw'] - self.args.goal_yaw),
                            math.cos(pose['yaw'] - self.args.goal_yaw))),
                        'speed_mps': pose['linear_x']}
                self.emit('settled_result', data=assessment)
                self.settling = False
                self.finish(1)
                return
            return
        if self.post_action_deadline_ns is not None:
            if now >= self.post_action_deadline_ns:
                self.post_action_deadline_ns = None
                self.finish(self.post_action_exit_code)
            return
        if self.finish_requested:
            if self.finish_deadline_ns is None or now >= self.finish_deadline_ns:
                self.done = True
            return
        if now >= self.deadline_ns:
            self.emit('recorder_timeout', data={'action_status': self.action_status})
            if self.goal_handle is not None:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.on_timeout_cancel)
            self.finish(2)
            return
        if getattr(self, 'started_goal', False):
            return
        # Do not submit a goal until the simulated localization and obstacle
        # sensor have both delivered fresh samples. In a quick blocked-route
        # abort this also ensures standstill evidence exists before Nav2 can
        # return its result; otherwise discovery/startup jitter looks like
        # absent odometry rather than a verified stationary vehicle.
        if (self.last_odom_receive_ns is None or self.last_scan_receive_ns is None or
                now - self.last_odom_receive_ns > 300_000_000 or
                now - self.last_scan_receive_ns > 300_000_000):
            return
        if not self.action.server_is_ready():
            return
        if self.args.enable_gate:
            if not self.enable_client.service_is_ready():
                return
            self.started_goal = True
            future = self.enable_client.call_async(SetBool.Request(data=True))
            future.add_done_callback(self.on_gate_enabled)
        else:
            self.started_goal = True
            self.send_goal()

    def on_gate_enabled(self, future):
        try:
            response = future.result()
            self.emit('gate_enable_response', data={
                'success': response.success, 'message': response.message})
            if not response.success:
                self.finish(2)
                return
            self.gate_was_enabled = True
            self.send_goal()
        except Exception as exc:  # Keep the bounded capture's failure evidence.
            self.emit('gate_enable_exception', data={'error': str(exc)})
            self.finish(2)

    def send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.args.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.args.goal_x
        goal.pose.pose.position.y = self.args.goal_y
        goal.pose.pose.orientation.z = math.sin(self.args.goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.args.goal_yaw / 2.0)
        self.emit('goal_sent', data={
            'frame': self.args.frame, 'x': self.args.goal_x,
            'y': self.args.goal_y, 'yaw': self.args.goal_yaw})
        future = self.action.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        try:
            self.goal_handle = future.result()
            self.emit('goal_response', data={'accepted': self.goal_handle.accepted})
            if not self.goal_handle.accepted:
                self.finish(1)
                return
            self.goal_handle.get_result_async().add_done_callback(self.on_result)
        except Exception as exc:
            self.emit('goal_response_exception', data={'error': str(exc)})
            self.finish(2)

    def on_feedback(self, feedback):
        receive_mono_ns = time.monotonic_ns()
        if (receive_mono_ns - self.last_feedback_mono_ns
                < self.feedback_period_ns):
            return
        self.last_feedback_mono_ns = receive_mono_ns
        value = feedback.feedback
        pose = value.current_pose.pose
        self.emit('action_feedback', topic='/navigate_to_pose',
                  header=value.current_pose.header, data={
                      'x': pose.position.x, 'y': pose.position.y,
                      'yaw': yaw_from_quaternion(pose.orientation),
                      'navigation_time_sec': (value.navigation_time.sec +
                                              value.navigation_time.nanosec * 1e-9),
                      'distance_remaining_m': value.distance_remaining,
                      'number_of_recoveries': value.number_of_recoveries},
                  receive_mono_ns=receive_mono_ns)

    def on_result(self, future):
        try:
            wrapped = future.result()
            self.action_status = int(wrapped.status)
            self.emit('action_result', topic='/navigate_to_pose', data={
                'status': self.action_status,
                'status_name': {4: 'SUCCEEDED', 5: 'CANCELED',
                                6: 'ABORTED'}.get(self.action_status, 'UNKNOWN')})
            if self.action_status == 4:
                self.settling = True
                self.settle_deadline_ns = time.monotonic_ns() + int(
                    self.args.settle_timeout * 1e9)
                self.emit('settling_started', data={
                    'timeout_sec': self.args.settle_timeout,
                    'goal_xy_tolerance_m': self.args.goal_xy_tolerance,
                    'goal_yaw_tolerance_rad': self.args.goal_yaw_tolerance,
                    'stopped_speed_mps': self.args.stopped_speed})
            else:
                # Keep subscribers alive briefly after a bounded abort so the
                # evidence includes stop output, odometry and gate state.
                self.post_action_exit_code = 1
                self.post_action_deadline_ns = time.monotonic_ns() + 1_000_000_000
        except Exception as exc:
            self.emit('action_result_exception', data={'error': str(exc)})
            self.post_action_exit_code = 2
            self.post_action_deadline_ns = time.monotonic_ns() + 1_000_000_000

    def on_timeout_cancel(self, future):
        try:
            response = future.result()
            self.emit('timeout_cancel_response', data={
                'goals_canceling': len(response.goals_canceling)})
        except Exception as exc:
            self.emit('timeout_cancel_exception', data={'error': str(exc)})

    def finish(self, exit_code):
        if self.done or self.finish_requested:
            return
        self.finish_requested = True
        self.exit_code = exit_code
        self.emit('capture_finished', data={
            'exit_code': exit_code, 'action_status': self.action_status,
            'elapsed_mono_sec': (time.monotonic_ns() - self.started_mono_ns) * 1e-9})
        if self.gate_was_enabled and self.enable_client.service_is_ready():
            self.finish_deadline_ns = time.monotonic_ns() + 2_000_000_000
            future = self.enable_client.call_async(SetBool.Request(data=False))
            future.add_done_callback(self.on_gate_disabled)
        else:
            self.finish_deadline_ns = None
            self.done = True
        self.stream.flush()

    def on_gate_disabled(self, future):
        try:
            response = future.result()
            self.emit('gate_disable_response', data={
                'success': response.success, 'message': response.message})
        except Exception as exc:
            self.emit('gate_disable_exception', data={'error': str(exc)})
        self.finish_deadline_ns = None
        self.done = True

    def on_path(self, msg):
        start = time.monotonic_ns()
        poses = []
        for stamped in msg.poses:
            p = stamped.pose
            poses.append({'x': p.position.x, 'y': p.position.y,
                          'yaw': yaw_from_quaternion(p.orientation)})
        self.emit('path', '/plan', msg.header, {'poses': poses}, start, start)

    def on_map(self, msg):
        start = time.monotonic_ns()
        self.emit('occupancy_grid', '/map', msg.header, {
            'resolution': msg.info.resolution,
            'width': msg.info.width, 'height': msg.info.height,
            'origin': {'x': msg.info.origin.position.x,
                       'y': msg.info.origin.position.y,
                       'yaw': yaw_from_quaternion(msg.info.origin.orientation)},
            'occupancy': list(msg.data)}, start, start)

    def on_rosout(self, msg):
        start = time.monotonic_ns()
        self.emit('rosout', '/rosout', data={
            'stamp': stamp_dict(msg.stamp), 'name': msg.name,
            'level': int(msg.level), 'message': msg.msg,
            'file': msg.file, 'function': msg.function, 'line': int(msg.line)},
            receive_mono_ns=start, callback_start_ns=start)

    def on_nav_cmd(self, msg):
        self.record_twist('/atlas_ackermann/cmd_vel_nav', msg)

    def on_safe_cmd(self, msg):
        self.record_twist('/atlas_ackermann/cmd_vel_safe', msg)

    def record_twist(self, topic, msg):
        start = time.monotonic_ns()
        self.emit('twist', topic, data={
            'linear_x': finite(msg.linear.x),
            'angular_z': finite(msg.angular.z)},
            receive_mono_ns=start, callback_start_ns=start)

    def on_adapter_status(self, msg):
        self.record_json_status('/atlas_ackermann/command_status', msg)

    def on_control(self, msg):
        start = time.monotonic_ns()
        self.emit('control_command', '/atlas_ackermann/control_cmd',
                  data={
                      'stamp': stamp_dict(msg.stamp),
                      'longitudinal_velocity_mps': finite(msg.longitudinal.velocity),
                      'longitudinal_acceleration_mps2': finite(msg.longitudinal.acceleration),
                      'steering_angle_rad': finite(msg.lateral.steering_tire_angle),
                      'steering_rate_radps': finite(msg.lateral.steering_tire_rotation_rate),
                      'steering_rate_defined': bool(msg.lateral.is_defined_steering_tire_rotation_rate),
                  }, receive_mono_ns=start,
                  callback_start_ns=start)

    def on_simulator_dynamics(self, msg):
        start = time.monotonic_ns()
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            data = {'invalid_json': True, 'raw': msg.data}
        self.emit('simulator_dynamics', '/atlas_ackermann/simulator_dynamics',
                  data=data, receive_mono_ns=start, callback_start_ns=start)

    def on_gate_status(self, msg):
        self.record_json_status('/atlas_nav2/safety_status', msg)

    def record_json_status(self, topic, msg):
        start = time.monotonic_ns()
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            data = {'raw': msg.data}
        self.emit('status', topic, data=data,
                  receive_mono_ns=start, callback_start_ns=start)

    def on_odom(self, msg):
        start = time.monotonic_ns()
        self.last_odom_receive_ns = start
        pose = msg.pose.pose
        self.last_odom = {
            'x': pose.position.x, 'y': pose.position.y,
            'yaw': yaw_from_quaternion(pose.orientation),
            'linear_x': finite(msg.twist.twist.linear.x),
            'angular_z': finite(msg.twist.twist.angular.z)}
        self.emit('odometry', '/atlas_ackermann/odom', msg.header, {
            'child_frame_id': msg.child_frame_id,
            'x': pose.position.x, 'y': pose.position.y,
            'yaw': yaw_from_quaternion(pose.orientation),
            'linear_x': finite(msg.twist.twist.linear.x),
            'angular_z': finite(msg.twist.twist.angular.z)}, start, start)

    def on_scan(self, msg):
        start = time.monotonic_ns()
        self.last_scan_receive_ns = start
        self.emit('laser_scan', '/atlas_ackermann/scan', msg.header, {
            'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment, 'scan_time': msg.scan_time,
            'range_min': msg.range_min, 'range_max': msg.range_max,
            'ranges': [finite(r) for r in msg.ranges]}, start, start)

    def on_tf(self, msg):
        self.record_tf('/tf', msg)

    def on_tf_static(self, msg):
        self.record_tf('/tf_static', msg)

    def record_tf(self, topic, msg):
        start = time.monotonic_ns()
        transforms = []
        for item in msg.transforms:
            t = item.transform
            transforms.append({
                'parent': item.header.frame_id, 'child': item.child_frame_id,
                'stamp': stamp_dict(item.header.stamp),
                'translation': [t.translation.x, t.translation.y, t.translation.z],
                'rotation': [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]})
        self.emit('transforms', topic, data=transforms,
                  receive_mono_ns=start, callback_start_ns=start)

    def on_local_costmap(self, msg):
        self.record_costmap('/local_costmap/costmap', msg)

    def on_local_costmap_raw(self, msg):
        self.record_costmap('/local_costmap/costmap_raw', msg)

    def on_global_costmap(self, msg):
        self.record_costmap('/global_costmap/costmap', msg)

    def on_global_costmap_raw(self, msg):
        self.record_costmap('/global_costmap/costmap_raw', msg)

    def record_costmap(self, topic, msg):
        start = time.monotonic_ns()
        metadata = msg.metadata
        origin = metadata.origin
        data = {
            'layer': metadata.layer,
            'resolution': metadata.resolution,
            'size_x': metadata.size_x,
            'size_y': metadata.size_y,
            'origin': {'x': origin.position.x, 'y': origin.position.y,
                       'yaw': yaw_from_quaternion(origin.orientation)},
            'costs': list(msg.data),
        }
        self.emit('costmap', topic, msg.header, data, start, start)

    def on_local_footprint(self, msg):
        self.record_footprint('/local_costmap/published_footprint', msg)

    def on_global_footprint(self, msg):
        self.record_footprint('/global_costmap/published_footprint', msg)

    def record_footprint(self, topic, msg):
        start = time.monotonic_ns()
        points = [{'x': point.x, 'y': point.y, 'z': point.z}
                  for point in msg.polygon.points]
        self.emit('footprint', topic, msg.header, {'points': points}, start, start)

    def close(self):
        self.stream.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True,
                        help='new JSONL path; existing files are never overwritten')
    parser.add_argument('--goal-x', type=float, required=True)
    parser.add_argument('--goal-y', type=float, required=True)
    parser.add_argument('--goal-yaw', type=float, default=0.0)
    parser.add_argument('--frame', default='map')
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--settle-timeout', type=float, default=3.0,
                        help='post-action time allowed to verify standstill and goal error')
    parser.add_argument('--goal-xy-tolerance', type=float, default=0.20)
    parser.add_argument('--goal-yaw-tolerance', type=float, default=0.25)
    parser.add_argument('--stopped-speed', type=float, default=0.02)
    parser.add_argument('--enable-gate', action='store_true',
                        help='enable the isolated simulation safety gate before sending the goal')
    args = parser.parse_args(argv)
    if (args.timeout <= 0.0 or args.settle_timeout <= 0.0 or
            args.goal_xy_tolerance <= 0.0 or args.goal_yaw_tolerance <= 0.0 or
            args.stopped_speed < 0.0):
        parser.error('timeouts and goal tolerances must be positive; stopped speed must be nonnegative')
    rclpy.init()
    node = AckermannRunRecorder(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code if node.done else 2
    except KeyboardInterrupt:
        node.emit('capture_interrupted')
        return 130
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
