#!/usr/bin/env python3
"""Bounded, evidence-producing Nav2 closed-loop validation."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


class Monitor(Node):
    def __init__(self):
        super().__init__('atlas_nav2_closed_loop_validator')
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.map = self.global_costmap = self.local_costmap = None
        self.odom = []
        self.nav_commands = []
        self.safe_commands = []
        self.wheels = []
        self.scans = []
        self.safety = []
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), transient)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'global_costmap', m), 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: setattr(self, 'local_costmap', m), 10)
        self.create_subscription(Odometry, '/atlas_nav2/odom', self.on_odom, 50)
        self.create_subscription(Twist, '/atlas_nav2/cmd_vel_nav',
                                 lambda m: self.nav_commands.append(self.twist(m)), 50)
        self.create_subscription(Twist, '/atlas_nav2/cmd_vel_safe',
                                 lambda m: self.safe_commands.append(self.twist(m)), 50)
        self.create_subscription(JointState, '/atlas_nav2/wheel_speed_targets',
                                 lambda m: self.wheels.append((time.monotonic(), list(m.velocity))), 50)
        self.create_subscription(LaserScan, '/atlas_nav2/scan', self.on_scan, 20)
        self.create_subscription(String, '/atlas_nav2/safety_status', self.on_safety, 20)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.enable = self.create_client(SetBool, '/safety_gate/enable')
        self.lifecycle = {name: self.create_client(GetState, f'/{name}/get_state') for name in (
            'map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator')}

    @staticmethod
    def twist(msg):
        return time.monotonic(), msg.linear.x, msg.angular.z

    def on_odom(self, msg):
        self.odom.append((time.monotonic(), msg.pose.pose.position.x,
                          msg.pose.pose.position.y, msg.twist.twist.linear.x,
                          msg.twist.twist.angular.z, msg.header.frame_id,
                          msg.child_frame_id))

    def on_scan(self, msg):
        finite = [value for value in msg.ranges if math.isfinite(value)]
        self.scans.append((time.monotonic(), min(finite, default=msg.range_max), msg.header.frame_id))

    def on_safety(self, msg):
        try:
            self.safety.append((time.monotonic(), json.loads(msg.data)))
        except ValueError:
            self.safety.append((time.monotonic(), {'invalid_json': True}))


def spin_until(node, predicate, timeout, launch):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
        if launch.poll() is not None:
            raise RuntimeError('launch exited unexpectedly')
    return False


def call(node, client, request, timeout, launch):
    if not spin_until(node, lambda: client.service_is_ready(), timeout, launch):
        raise RuntimeError(f'service unavailable: {client.srv_name}')
    future = client.call_async(request)
    if not spin_until(node, future.done, timeout, launch):
        raise RuntimeError(f'service timed out: {client.srv_name}')
    return future.result()


def stop_group(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    # This first closed-loop acceptance goal stays in open map space. Obstacle
    # avoidance is deliberately tracked as a separate, not-yet-passing test.
    parser.add_argument('--goal-x', type=float, default=0.0)
    parser.add_argument('--goal-y', type=float, default=1.0)
    parser.add_argument('--timeout-sec', type=float, default=100.0)
    args = parser.parse_args()
    if not 30.0 <= args.timeout_sec <= 180.0:
        parser.error('--timeout-sec must be 30..180')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'ros_logs').mkdir()
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
    report = {'result': 'RUNNING', 'goal': [args.goal_x, args.goal_y], 'checks': {},
              'simulation_assumptions': ['ideal_no_slip_kinematics', 'identity_map_to_odom']}
    (output / 'summary.json').write_text(json.dumps(report, indent=2))
    launch = None
    node = None
    log = (output / 'launch.log').open('w')
    try:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'atlas_nav2_demo', 'nav2_skid_steer.launch.py'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=os.environ.copy())
        rclpy.init()
        node = Monitor()

        def lifecycle_active():
            states = {}
            for name, client in node.lifecycle.items():
                if not client.service_is_ready():
                    return False
                future = client.call_async(GetState.Request())
                if not spin_until(node, future.done, 2.0, launch):
                    return False
                states[name] = future.result().current_state.id
            report['lifecycle_states'] = states
            return all(value == State.PRIMARY_STATE_ACTIVE for value in states.values())

        if not spin_until(node, lifecycle_active, 60.0, launch):
            raise RuntimeError('required Nav2 lifecycle nodes did not become active')
        report['checks']['lifecycle_active'] = True
        if not spin_until(node, lambda: node.map is not None and node.global_costmap is not None
                          and node.local_costmap is not None and len(node.scans) >= 5
                          and len(node.odom) >= 10, 20.0, launch):
            raise RuntimeError('map, costmap, scan, or odometry data missing')
        report['checks']['map_and_costmaps'] = True
        if not node.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(),
                                            timeout=Duration(seconds=2.0)):
            raise RuntimeError('TF map -> base_link unavailable')
        if not node.tf_buffer.can_transform('base_link', 'laser', rclpy.time.Time(),
                                            timeout=Duration(seconds=2.0)):
            raise RuntimeError('TF base_link -> laser unavailable')
        report['checks']['tf_tree'] = True

        pre_enable = len(node.safe_commands)
        spin_until(node, lambda: len(node.safe_commands) >= pre_enable + 5, 2.0, launch)
        if any(abs(v) > 1e-9 or abs(w) > 1e-9 for _, v, w in node.safe_commands[pre_enable:]):
            raise RuntimeError('default-disabled gate emitted nonzero command')
        report['checks']['gate_default_disabled'] = True
        enabled = call(node, node.enable, SetBool.Request(data=True), 5.0, launch)
        if not enabled.success:
            raise RuntimeError('simulation safety gate refused explicit enable')
        report['checks']['gate_explicitly_enabled'] = True

        if not spin_until(node, node.navigator.server_is_ready, 10.0, launch):
            raise RuntimeError('NavigateToPose action unavailable')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.goal_x
        goal.pose.pose.position.y = args.goal_y
        goal.pose.pose.orientation.w = 1.0
        sent = node.navigator.send_goal_async(goal)
        if not spin_until(node, sent.done, 10.0, launch) or not sent.result().accepted:
            raise RuntimeError('navigation goal was rejected')
        handle = sent.result()
        result_future = handle.get_result_async()
        started = time.monotonic()
        if not spin_until(node, result_future.done, args.timeout_sec, launch):
            cancel = handle.cancel_goal_async()
            spin_until(node, cancel.done, 5.0, launch)
            raise RuntimeError('navigation goal exceeded bounded timeout')
        result_status = result_future.result().status
        report['action_status'] = result_status
        if result_status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'navigation action did not succeed: status {result_status}')
        report['checks']['goal_succeeded'] = True

        after_result = len(node.safe_commands)
        if not spin_until(node, lambda: len(node.safe_commands) >= after_result + 10, 3.0, launch):
            raise RuntimeError('safe command stream stopped after goal')
        if any(abs(v) > 0.01 or abs(w) > 0.01 for _, v, w in node.safe_commands[-5:]):
            raise RuntimeError('robot command did not return to zero after completion')
        report['checks']['zero_after_completion'] = True
        disabled = call(node, node.enable, SetBool.Request(data=False), 5.0, launch)
        if not disabled.success:
            raise RuntimeError('simulation gate disable failed')
        disabled_index = len(node.safe_commands)
        spin_until(node, lambda: len(node.safe_commands) >= disabled_index + 5, 2.0, launch)
        if any(abs(v) > 1e-9 or abs(w) > 1e-9 for _, v, w in node.safe_commands[disabled_index:]):
            raise RuntimeError('disabled gate did not hold zero output')
        report['checks']['gate_disabled_zero'] = True

        initial = node.odom[0]
        final = node.odom[-1]
        displacement = math.hypot(final[1] - initial[1], final[2] - initial[2])
        goal_error = math.hypot(final[1] - args.goal_x, final[2] - args.goal_y)
        nonzero_nav = any(abs(v) > 0.02 or abs(w) > 0.02 for _, v, w in node.nav_commands)
        nonzero_safe = any(abs(v) > 0.02 or abs(w) > 0.02 for _, v, w in node.safe_commands)
        nonzero_wheels = any(any(abs(value) > 0.1 for value in values) for _, values in node.wheels)
        local_obstacles = sum(value >= 99 for value in node.local_costmap.data)
        metrics = {
            'duration_sec': time.monotonic() - started,
            'displacement_m': displacement,
            'goal_error_m': goal_error,
            'nav_command_samples': len(node.nav_commands),
            'safe_command_samples': len(node.safe_commands),
            'wheel_target_samples': len(node.wheels),
            'odometry_samples': len(node.odom),
            'scan_samples': len(node.scans),
            'minimum_scan_range_m': min(value[1] for value in node.scans),
            'local_costmap_occupied_cells': local_obstacles,
        }
        report['metrics'] = metrics
        assertions = {
            'nonzero_nav_commands': nonzero_nav,
            'nonzero_safe_commands': nonzero_safe,
            'nonzero_wheel_targets': nonzero_wheels,
            'simulated_displacement': displacement > 0.7,
            'goal_approach': goal_error < 0.35,
            'odometry_frames': all(sample[5:] == ('odom', 'base_link') for sample in node.odom),
            'laser_frame': all(sample[2] == 'laser' for sample in node.scans),
        }
        report['checks'].update(assertions)
        if not all(assertions.values()):
            raise RuntimeError('one or more closed-loop assertions failed')
        report['result'] = 'PASS'
    except KeyboardInterrupt:
        report['result'] = 'INTERRUPTED'
        report['failure'] = 'operator interrupted validation'
    except Exception as exc:
        report['result'] = 'FAIL'
        report['failure'] = str(exc)
    finally:
        if node is not None:
            report.setdefault('final', {})
            report['final']['odom'] = node.odom[-1] if node.odom else None
            report['final']['safety'] = node.safety[-1] if node.safety else None
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        stop_group(launch)
        log.close()
        (output / 'summary.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
