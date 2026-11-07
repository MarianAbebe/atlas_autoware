#!/usr/bin/env python3
"""Extend the existing simulation latch with accepted NDT measurement freshness.

EKF prediction output timestamps alone cannot detect lost absolute localization.
No vehicle controls are published here; the existing pinned gate brakes.
"""
import importlib.util
import json
from pathlib import Path
import time
from collections import defaultdict
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_internal_debug_msgs.msg import Float32Stamped

path = Path(get_package_prefix('atlas_closed_loop_demo')) / 'lib/atlas_closed_loop_demo/feedback_watchdog.py'
spec = importlib.util.spec_from_file_location('atlas_existing_watchdog', path)
existing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(existing)


class LocalizationWatchdog(existing.Watchdog):
    def __init__(self):
        super().__init__()
        self.callback_counts = defaultdict(int)
        self.callback_gaps = {}
        self.callback_last_receipt = {}
        self.callback_last_source_age = {}
        self.callback_max_duration = {}
        self.callback_last_duration = {}
        self.tick_gap_sec = None
        self._diagnostic_last_tick = None
        self._diagnostic_tick_start = None
        self.ndt_callback_group = MutuallyExclusiveCallbackGroup()
        self.ndt_runtime_count = 0
        self.ndt_runtime_last_receipt_monotonic = None
        self.ndt_runtime_max_receipt_gap_sec = 0.0
        self.ndt_runtime_last_source_stamp = None
        self.ndt_runtime_last_source_age_sec = None
        self.ndt_runtime_last_duration_sec = None
        self.ndt_runtime_max_duration_sec = 0.0
        self.ndt_runtime_last_ms = None
        self.create_subscription(PoseWithCovarianceStamped, 'ndt/ndt_pose_with_covariance',
                                 lambda msg: self.receive('ndt', msg), 10,
                                 callback_group=self.ndt_callback_group)
        self.create_subscription(Float32Stamped, 'ndt/exe_time_ms', self.ndt_runtime, 10,
                                 callback_group=self.ndt_callback_group)

    def ndt_runtime(self, msg):
        """Observe NDT matcher activity; this stream never affects watchdog decisions."""
        start = time.monotonic()
        source = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        with self.state_lock:
            previous = self.ndt_runtime_last_receipt_monotonic
            if previous is not None:
                self.ndt_runtime_max_receipt_gap_sec = max(
                    self.ndt_runtime_max_receipt_gap_sec, start - previous)
            self.ndt_runtime_last_receipt_monotonic = start
            self.ndt_runtime_count += 1
            self.ndt_runtime_last_source_stamp = source
            self.ndt_runtime_last_source_age_sec = ros_now - source
            self.ndt_runtime_last_ms = float(msg.data)
        duration = time.monotonic() - start
        with self.state_lock:
            self.ndt_runtime_last_duration_sec = duration
            self.ndt_runtime_max_duration_sec = max(
                self.ndt_runtime_max_duration_sec, duration)

    def receive(self, name, msg):
        callback_start = time.monotonic()
        receipt = callback_start
        source_stamp = msg.header.stamp if hasattr(msg, 'header') else msg.stamp
        source = source_stamp.sec + source_stamp.nanosec * 1e-9
        super().receive(name, msg)
        callback_duration = time.monotonic()-callback_start
        with self.state_lock:
            previous = self.callback_last_receipt.get(name)
            if previous is not None:
                self.callback_gaps[name] = max(self.callback_gaps.get(name, 0.0), receipt - previous)
            self.callback_last_receipt[name] = receipt
            self.callback_counts[name] += 1
            self.callback_last_source_age[name] = self.get_clock().now().nanoseconds * 1e-9 - source
            self.callback_last_duration[name] = callback_duration
            self.callback_max_duration[name] = max(self.callback_max_duration.get(name, 0.0), callback_duration)

    def tick(self):
        start = time.monotonic()
        if self._diagnostic_last_tick is not None:
            self.tick_gap_sec = start - self._diagnostic_last_tick
        self._diagnostic_last_tick = start
        self._diagnostic_tick_start = start
        was_bad = self.bad
        super().tick()
        if not was_bad and self.bad:
            ndt = (self.fault_health or {}).get('ndt', {})
            self.get_logger().warn('watchdog fault diagnostics: '+json.dumps({
                'ndt_source_age_sec':ndt.get('stamp_age_sec'),
                'ndt_receipt_age_sec':ndt.get('receipt_age_sec'),
                'ndt_last_callback_monotonic_sec':self.callback_last_receipt.get('ndt'),
                'ndt_max_callback_gap_sec':self.callback_gaps.get('ndt'),
                'ndt_last_callback_duration_sec':self.callback_last_duration.get('ndt'),
                'ndt_max_callback_duration_sec':self.callback_max_duration.get('ndt'),
                'watchdog_tick_gap_sec':self.fault_tick_gap,
                'watchdog_process_cpu_time_sec':time.process_time()},separators=(',',':')))

    def publish_status(self, stale):
        # Add bounded counters and callback receipt timing to the existing status.
        # This does not alter freshness decisions or heartbeat behavior.
        from std_msgs.msg import String
        import json
        publish_start = time.monotonic()
        with self.state_lock:
            msg = String()
            msg.data = json.dumps({
                'stamp': self.get_clock().now().nanoseconds * 1e-9,
                'armed': self.ready and not self.bad, 'latched': self.bad,
                'fault_stamp': self.fault_at, 'fault_inputs': self.fault_inputs,
                'currently_stale': stale, 'stationary_since_monotonic': self.stationary_since,
                'fault_input_health': self.fault_health, 'fault_tick_gap_sec': self.fault_tick_gap,
                'instrumentation': {
                    'watchdog_tick_gap_sec': self.tick_gap_sec,
                    'watchdog_tick_start_monotonic_sec': self._diagnostic_tick_start,
                    'input_callback_counts': dict(self.callback_counts),
                    'input_callback_max_gap_sec': self.callback_gaps,
                    'input_last_callback_monotonic_sec': self.callback_last_receipt,
                    'input_last_source_age_at_callback_sec': self.callback_last_source_age,
                    'input_callback_last_duration_sec': self.callback_last_duration,
                    'input_callback_max_duration_sec': self.callback_max_duration,
                    'ndt_runtime_diagnostics': {
                        'count': self.ndt_runtime_count,
                        'last_receipt_monotonic_sec': self.ndt_runtime_last_receipt_monotonic,
                        'max_receipt_gap_sec': self.ndt_runtime_max_receipt_gap_sec,
                        'last_source_stamp': self.ndt_runtime_last_source_stamp,
                        'last_source_age_sec': self.ndt_runtime_last_source_age_sec,
                        'last_execution_ms': self.ndt_runtime_last_ms,
                        'last_callback_duration_sec': self.ndt_runtime_last_duration_sec,
                        'max_callback_duration_sec': self.ndt_runtime_max_duration_sec,
                    },
                    'publisher_timing': self.publication_diagnostics(publish_start),
                },
            }, separators=(',', ':'))
        # Diagnostic publication can enter the RMW. Keep NDT/input receive
        # callbacks outside that synchronous operation and retain the timer's
        # state snapshot above.
        publish_call_start = time.monotonic()
        self.status.publish(msg)
        publish_end = time.monotonic()
        self.status_publish_count += 1
        self.last_status_publish_monotonic = publish_start
        self.last_status_publish_call_start_monotonic = publish_call_start
        self.status_publish_duration_sec = publish_end - publish_call_start
        self.status_publish_operation_duration_sec = publish_end - publish_start

    def stale(self):
        with self.state_lock:
            bad = super().stale()
            now = self.get_clock().now().nanoseconds * 1e-9
            mono = time.monotonic()
            self.input_health['ndt'] = dict(source_stamp=self.stamps.get('ndt'),
                stamp_age_sec=now-self.stamps['ndt'] if 'ndt' in self.stamps else None,
                receipt_age_sec=mono-self.receipts['ndt'] if 'ndt' in self.receipts else None)
            if ('ndt' not in self.stamps or not 0 <= now-self.stamps['ndt'] < self.timeout
                    or mono-self.receipts['ndt'] >= self.timeout):
                bad.append('ndt')
            return bad


def main():
    rclpy.init()
    node = LocalizationWatchdog()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.shutdown_requested:
            executor.spin_once(timeout_sec=.02)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # ROS 2 Humble may invalidate a subscription handle while the launch
        # process group is shutting down. Treat this as expected only after the
        # context has stopped; preserve every runtime failure while it is active.
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
