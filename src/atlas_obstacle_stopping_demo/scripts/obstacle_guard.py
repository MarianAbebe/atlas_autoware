#!/usr/bin/env python3
"""Simulation-only authorization latch. Pinned gate supplies all fault braking."""
import json
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import PredictedObjects
from autoware_adapi_v1_msgs.msg import ManualOperatorHeartbeat
from std_msgs.msg import String
from std_srvs.srv import Trigger


def stamp(s):
    return s.sec + s.nanosec * 1e-9


class Guard(Node):
    def __init__(self):
        super().__init__('obstacle_guard')
        # Keep the safety timer and base watchdog authorization stream
        # serviceable independently from point-cloud/object callbacks.
        self.state_lock = threading.RLock()
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self.base_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.timeout = self.declare_parameter('freshness_sec', 0.3).value
        self.ready = False
        self.bad = False
        self.fault_at = None
        self.fault_inputs = []
        self.fault_health = None
        self.input_health = {}
        self.good_since = None
        self.stationary_since = None
        self.last_actual = None
        self.base = None
        self.samples = {}
        self.base_callback_counts = {'base_status': 0, 'base_heartbeat': 0}
        self.base_callback_last_receipt_monotonic = {}
        self.base_callback_max_gap_sec = {}
        self.base_callback_last_duration_sec = {}
        self.base_callback_max_duration_sec = {}
        self.base_callback_source_age_sec = {}
        self.shutdown_requested = False
        self.pub = self.create_publisher(ManualOperatorHeartbeat, 'gate/input/external_emergency_stop_heartbeat', 10)
        self.status = self.create_publisher(String, 'safety_status', 10)
        self.create_subscription(PointCloud2, 'obstacles/pointcloud', lambda m: self.receive('pointcloud', m.header.stamp), qos_profile_sensor_data,
                                 callback_group=self.sensor_callback_group)
        self.create_subscription(PredictedObjects, 'obstacles/predicted_objects', lambda m: self.receive('predicted_objects', m.header.stamp), 10,
                                 callback_group=self.sensor_callback_group)
        self.create_subscription(ManualOperatorHeartbeat, 'base_safety_heartbeat', self.heartbeat, 10,
                                 callback_group=self.base_callback_group)
        self.create_subscription(String, 'base_safety_status', self.base_status, 10,
                                 callback_group=self.base_callback_group)
        self.create_subscription(Odometry, 'sim/output/odometry', self.actual, 10,
                                 callback_group=self.sensor_callback_group)
        self.create_subscription(String, 'test_shutdown', lambda _: setattr(self, 'shutdown_requested', True), 10,
                                 callback_group=self.sensor_callback_group)
        self.create_service(Trigger, '~/rearm', self.rearm, callback_group=self.base_callback_group)
        self.create_timer(0.02, self.tick, callback_group=self.timer_callback_group)

    def record_base_callback(self, name, source_stamp, receipt, start):
        """Record callback delivery in this process monotonic clock domain."""
        end = time.monotonic()
        duration = end - start
        with self.state_lock:
            previous = self.base_callback_last_receipt_monotonic.get(name)
            if previous is not None:
                self.base_callback_max_gap_sec[name] = max(
                    self.base_callback_max_gap_sec.get(name, 0.0), receipt - previous)
            self.base_callback_last_receipt_monotonic[name] = receipt
            self.base_callback_counts[name] = self.base_callback_counts.get(name, 0) + 1
            self.base_callback_last_duration_sec[name] = duration
            self.base_callback_max_duration_sec[name] = max(
                self.base_callback_max_duration_sec.get(name, 0.0), duration)
            source_seconds = (float(source_stamp) if isinstance(source_stamp, (int, float))
                              else stamp(source_stamp))
            self.base_callback_source_age_sec[name] = (
                self.get_clock().now().nanoseconds * 1e-9 - source_seconds)

    def receive(self, name, source_stamp):
        with self.state_lock:
            self.samples[name] = (stamp(source_stamp), time.monotonic())

    def base_status(self, msg):
        start = time.monotonic()
        receipt = start
        try:
            base = json.loads(msg.data)
            source_stamp = base['stamp']
            with self.state_lock:
                self.base = base
                self.samples['base_status'] = (source_stamp, receipt)
        finally:
            if 'source_stamp' in locals():
                self.record_base_callback('base_status', source_stamp, receipt, start)

    def heartbeat(self, msg):
        start = time.monotonic()
        receipt = start
        if msg.ready:
            with self.state_lock:
                self.samples['base_heartbeat'] = (stamp(msg.stamp), receipt)
            self.record_base_callback('base_heartbeat', msg.stamp, receipt, start)

    def actual(self, msg):
        with self.state_lock:
            self.last_actual = time.monotonic()
            if abs(msg.twist.twist.linear.x) < 0.001:
                if self.stationary_since is None:
                    self.stationary_since = self.last_actual
            else:
                self.stationary_since = None

    def stale(self):
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        mono = time.monotonic()
        with self.state_lock:
            self.input_health = {n: dict(source_stamp=self.samples[n][0],
                stamp_age_sec=ros_now-self.samples[n][0], receipt_age_sec=mono-self.samples[n][1])
                if n in self.samples else dict(source_stamp=None, stamp_age_sec=None, receipt_age_sec=None)
                for n in ('pointcloud', 'predicted_objects', 'base_heartbeat', 'base_status')}
            bad = [n for n in ('pointcloud', 'predicted_objects', 'base_heartbeat', 'base_status')
                   if n not in self.samples or not 0 <= ros_now-self.samples[n][0] < self.timeout
                   or mono-self.samples[n][1] >= self.timeout]
            if self.base is None or not self.base['armed'] or self.base['latched']:
                bad.append('base_authorization')
            return bad

    def rearm(self, _, response):
        with self.state_lock:
            now = time.monotonic()
            stopped = (self.stationary_since is not None and now-self.stationary_since >= 3
                       and self.last_actual is not None and now-self.last_actual < self.timeout)
            response.success = self.bad and not self.stale() and stopped
            if response.success:
                self.bad = False
                self.ready = False
                self.good_since = None
                self.fault_at = None
                self.fault_inputs = []
                response.message = 'Explicit rearm accepted; separately clear the pinned gate emergency.'
            else:
                response.message = 'Require a latched fault, all fresh inputs, base authorization and 3 s of live standstill.'
        return response

    def tick(self):
        with self.state_lock:
            stale = self.stale()
            now = self.get_clock().now()
            if self.ready and stale and not self.bad:
                self.bad = True
                self.fault_at = now.nanoseconds * 1e-9
                self.fault_inputs = stale
                self.fault_health = dict(self.input_health)
            if stale:
                self.good_since = None
            elif not self.bad:
                if self.good_since is None:
                    self.good_since = time.monotonic()
                if time.monotonic()-self.good_since >= 1:
                    self.ready = True
                    msg = ManualOperatorHeartbeat()
                    msg.stamp = now.to_msg()
                    msg.ready = True
                    self.pub.publish(msg)
            msg = String()
            msg.data = json.dumps({'stamp': now.nanoseconds*1e-9, 'armed': self.ready and not self.bad,
                                   'latched': self.bad, 'fault_stamp': self.fault_at,
                                   'fault_inputs': self.fault_inputs, 'currently_stale': stale,
                                   'fault_input_health': self.fault_health,
                                   'current_input_health': self.input_health,
                                   'base_callback_diagnostics': {
                                       'callback_count': dict(self.base_callback_counts),
                                       'last_receipt_monotonic_sec': self.base_callback_last_receipt_monotonic,
                                       'max_receipt_gap_sec': self.base_callback_max_gap_sec,
                                       'last_callback_duration_sec': self.base_callback_last_duration_sec,
                                       'max_callback_duration_sec': self.base_callback_max_duration_sec,
                                       'source_age_at_callback_sec': self.base_callback_source_age_sec,
                                       'process_cpu_time_sec': time.process_time(),
                                   },
                                   'base_watchdog_state': (None if self.base is None else {
                                       'stamp': self.base.get('stamp'),
                                       'armed': self.base.get('armed'),
                                       'latched': self.base.get('latched'),
                                       'fault_inputs': self.base.get('fault_inputs'),
                                       'currently_stale': self.base.get('currently_stale'),
                                       'fault_tick_gap_sec': self.base.get('fault_tick_gap_sec'),
                                       'instrumentation': self.base.get('instrumentation', {}),
                                   })})
            self.status.publish(msg)


def main():
    rclpy.init()
    node = Guard()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.shutdown_requested:
            executor.spin_once(timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
