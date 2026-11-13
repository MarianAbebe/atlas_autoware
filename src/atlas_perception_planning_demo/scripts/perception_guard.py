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
from autoware_perception_msgs.msg import DetectedObjects, PredictedObjects
from autoware_adapi_v1_msgs.msg import ManualOperatorHeartbeat
from std_msgs.msg import String
from std_srvs.srv import Trigger


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def sample_health(samples, names, ros_now, mono_now):
    return {name: {'source_stamp': samples[name][0] if name in samples else None,
                   'stamp_age_sec': ros_now-samples[name][0] if name in samples else None,
                   'stamp_age_at_receipt_sec': samples[name][3] if name in samples else None,
                   'receipt_age_sec': mono_now-samples[name][1] if name in samples else None,
                   'receipt_gap_sec': samples[name][2] if name in samples else None}
            for name in names}


class Guard(Node):
    def __init__(self):
        super().__init__('obstacle_guard')
        # Authorization callbacks and the safety timer must remain serviceable
        # when the high-rate synthetic sensor inputs are active.
        self.state_lock = threading.RLock()
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self.predicted_callback_group = MutuallyExclusiveCallbackGroup()
        self.base_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.timeout = self.declare_parameter('freshness_sec', 0.3).value
        self.ready = False
        self.bad = False
        self.fault_at = None
        self.fault_inputs = []
        self.good_since = None
        self.stationary_since = None
        self.last_actual = None
        self.base = None
        self.samples = {}
        self.input_health = {}
        self.fault_health = None
        self.last_tick = None
        self.last_tick_duration = None
        self.last_tick_gap = None
        self.max_tick_gap = 0.0
        self.tick_count = 0
        self.fault_tick_gap = None
        self.base_callback_counts = {'base_status': 0, 'base_heartbeat': 0}
        self.base_callback_last_receipt_monotonic = {}
        self.base_callback_max_gap_sec = {}
        self.base_callback_last_duration_sec = {}
        self.base_callback_max_duration_sec = {}
        self.base_callback_source_age_sec = {}
        self.detected_callback_count = 0
        self.detected_callback_last_receipt_monotonic = None
        self.detected_callback_max_gap_sec = 0.0
        self.detected_callback_last_duration_sec = None
        self.detected_callback_max_duration_sec = 0.0
        self.detected_callback_last_source_stamp = None
        self.detected_callback_last_source_age_sec = None
        self.predicted_callback_count = 0
        self.predicted_callback_last_receipt_monotonic = None
        self.predicted_callback_max_gap_sec = 0.0
        self.predicted_callback_last_duration_sec = None
        self.predicted_callback_max_duration_sec = 0.0
        self.shutdown_requested = False
        self.pub = self.create_publisher(ManualOperatorHeartbeat, 'gate/input/external_emergency_stop_heartbeat', 10)
        self.status = self.create_publisher(String, 'safety_status', 10)
        # Monitor the raw acquisition stream and the final processed obstacle
        # cloud. The intervening converters preserve the original cloud
        # timestamp, and a stalled stage therefore ages the final output too.
        # Subscribing in Python to every full PointCloud2 payload only to read
        # its header caused redundant deserialization and scheduling cost.
        self.create_subscription(PointCloud2, 'obstacles/pointcloud', lambda m: self.receive('pointcloud', m.header.stamp), qos_profile_sensor_data,
                                 callback_group=self.sensor_callback_group)
        self.create_subscription(PointCloud2, 'lidar/raw', lambda m: self.receive('raw', m.header.stamp), qos_profile_sensor_data,
                                 callback_group=self.sensor_callback_group)
        self.create_subscription(DetectedObjects, 'obstacles/detected_objects',
                                 self.detected_objects, 10,
                                 callback_group=self.sensor_callback_group)
        self.create_subscription(PredictedObjects, 'obstacles/predicted_objects',
                                 self.predicted_objects, 10,
                                 callback_group=self.predicted_callback_group)
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
        """Record publisher-to-callback receipt diagnostics in local clock domains."""
        duration = time.monotonic() - start
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
        mono=time.monotonic();source=stamp(source_stamp);ros_now=self.get_clock().now().nanoseconds*1e-9
        with self.state_lock:
            previous=self.samples.get(name)
            self.samples[name] = (source, mono, None if previous is None else mono-previous[1], ros_now-source)

    def detected_objects(self, msg):
        """Observe detector output without making it a safety input."""
        callback_start = time.monotonic()
        source = stamp(msg.header.stamp)
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        with self.state_lock:
            previous = self.detected_callback_last_receipt_monotonic
            if previous is not None:
                self.detected_callback_max_gap_sec = max(
                    self.detected_callback_max_gap_sec, callback_start - previous)
            self.detected_callback_last_receipt_monotonic = callback_start
            self.detected_callback_count += 1
            self.detected_callback_last_source_stamp = source
            self.detected_callback_last_source_age_sec = ros_now - source
        duration = time.monotonic() - callback_start
        with self.state_lock:
            self.detected_callback_last_duration_sec = duration
            self.detected_callback_max_duration_sec = max(
                self.detected_callback_max_duration_sec, duration)

    def predicted_objects(self, msg):
        callback_start = time.monotonic()
        source = stamp(msg.header.stamp)
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        with self.state_lock:
            previous = self.samples.get('predicted_objects')
            previous_receipt = self.predicted_callback_last_receipt_monotonic
            if previous_receipt is not None:
                self.predicted_callback_max_gap_sec = max(
                    self.predicted_callback_max_gap_sec, callback_start-previous_receipt)
            self.predicted_callback_last_receipt_monotonic = callback_start
            self.predicted_callback_count += 1
            self.samples['predicted_objects'] = (
                source, callback_start,
                None if previous is None else callback_start-previous[1], ros_now-source)
        duration = time.monotonic() - callback_start
        with self.state_lock:
            self.predicted_callback_last_duration_sec = duration
            self.predicted_callback_max_duration_sec = max(
                self.predicted_callback_max_duration_sec, duration)

    def base_status(self, msg):
        start=time.monotonic();mono=start
        try:
            base=json.loads(msg.data);source=float(base['stamp']);ros_now=self.get_clock().now().nanoseconds*1e-9
            with self.state_lock:
                previous=self.samples.get('base_status')
                self.base=base
                self.samples['base_status']=(source,mono,None if previous is None else mono-previous[1],ros_now-source)
        finally:
            if 'source' in locals():
                self.record_base_callback('base_status',source,mono,start)

    def heartbeat(self, msg):
        start=time.monotonic();mono=start
        if msg.ready:
            with self.state_lock:
                source=stamp(msg.stamp);ros_now=self.get_clock().now().nanoseconds*1e-9
                previous=self.samples.get('base_heartbeat')
                self.samples['base_heartbeat']=(source,mono,None if previous is None else mono-previous[1],ros_now-source)
            self.record_base_callback('base_heartbeat',msg.stamp,mono,start)

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
        names = ('raw', 'pointcloud', 'predicted_objects', 'base_heartbeat', 'base_status')
        with self.state_lock:
            self.input_health = sample_health(self.samples, names, ros_now, mono)
            bad = [n for n in names
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
            was_bad = self.bad
            tick_now = time.monotonic()
            tick_gap = tick_now-self.last_tick if self.last_tick is not None else None
            self.last_tick = tick_now
            self.tick_count += 1
            self.last_tick_gap = tick_gap
            if tick_gap is not None:
                self.max_tick_gap = max(self.max_tick_gap, tick_gap)
            stale = self.stale()
            now = self.get_clock().now()
            heartbeat_msg = None
            if self.ready and stale and not self.bad:
                self.bad = True
                self.fault_at = now.nanoseconds * 1e-9
                self.fault_inputs = stale
                self.fault_health = dict(self.input_health)
                self.fault_tick_gap = tick_gap
            if stale:
                self.good_since = None
            elif not self.bad:
                if self.good_since is None:
                    self.good_since = time.monotonic()
                if time.monotonic()-self.good_since >= 1:
                    self.ready = True
                    heartbeat_msg = ManualOperatorHeartbeat()
                    heartbeat_msg.stamp = now.to_msg()
                    heartbeat_msg.ready = True
                else:
                    heartbeat_msg = None
            else:
                heartbeat_msg = None
            if heartbeat_msg is not None:
                # The authorization decision and heartbeat publication remain
                # atomic with respect to input freshness updates.
                self.pub.publish(heartbeat_msg)
            self.last_tick_duration = time.monotonic()-tick_now
            publish_diagnostic = self.tick_count % 5 == 0 or (self.bad and not was_bad)
            if publish_diagnostic:
                diagnostic = {'stamp': now.nanoseconds*1e-9, 'armed': self.ready and not self.bad,
                              'latched': self.bad, 'fault_stamp': self.fault_at,
                              'fault_inputs': list(self.fault_inputs), 'currently_stale': stale,
                              'fault_input_health': self.fault_health,
                              'fault_tick_gap_sec': self.fault_tick_gap,
                              'instrumentation': {'tick_count': self.tick_count,
                                                  'tick_schedule_gap_sec': tick_gap,
                                                  'tick_max_schedule_gap_sec': self.max_tick_gap,
                                                  'tick_work_sec': self.last_tick_duration,
                                                  'inputs': dict(self.input_health),
                                                  'base_callback_diagnostics': {
                                                      'callback_count': dict(self.base_callback_counts),
                                                      'last_receipt_monotonic_sec': dict(self.base_callback_last_receipt_monotonic),
                                                      'max_receipt_gap_sec': dict(self.base_callback_max_gap_sec),
                                                      'last_callback_duration_sec': dict(self.base_callback_last_duration_sec),
                                                      'max_callback_duration_sec': dict(self.base_callback_max_duration_sec),
                                                      'source_age_at_callback_sec': dict(self.base_callback_source_age_sec),
                                                      'process_cpu_time_sec': time.process_time(),
                                                  },
                                                  'detected_objects_callback': {
                                                      'count': self.detected_callback_count,
                                                      'last_receipt_monotonic_sec': self.detected_callback_last_receipt_monotonic,
                                                      'max_receipt_gap_sec': self.detected_callback_max_gap_sec,
                                                      'last_source_stamp': self.detected_callback_last_source_stamp,
                                                      'last_source_age_sec': self.detected_callback_last_source_age_sec,
                                                      'last_callback_duration_sec': self.detected_callback_last_duration_sec,
                                                      'max_callback_duration_sec': self.detected_callback_max_duration_sec,
                                                  },
                                                  'predicted_objects_callback': {
                                                      'count': self.predicted_callback_count,
                                                      'last_receipt_monotonic_sec': self.predicted_callback_last_receipt_monotonic,
                                                      'max_receipt_gap_sec': self.predicted_callback_max_gap_sec,
                                                      'last_callback_duration_sec': self.predicted_callback_last_duration_sec,
                                                      'max_callback_duration_sec': self.predicted_callback_max_duration_sec,
                                                  },
                                                  'base_watchdog_state': (None if self.base is None else {
                                                      'stamp': self.base.get('stamp'),
                                                      'armed': self.base.get('armed'),
                                                      'latched': self.base.get('latched'),
                                                      'fault_inputs': self.base.get('fault_inputs'),
                                                      'currently_stale': self.base.get('currently_stale'),
                                                      'instrumentation': self.base.get('instrumentation', {}),
                                                  })}}

        # Keep serialization and DDS publication outside the state lock so
        # base status/heartbeat callbacks are not blocked by diagnostics.
        # Status is recorder-facing diagnostics; heartbeat authorization and
        # safety decisions above remain at 50 Hz. Avoid repeatedly encoding
        # the nested callback/timing snapshot on every safety tick.
        if publish_diagnostic:
            msg = String()
            msg.data = json.dumps(diagnostic, separators=(',', ':'))
            self.status.publish(msg)


def main():
    rclpy.init()
    node = Guard()
    executor = MultiThreadedExecutor(num_threads=4)
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
