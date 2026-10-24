#!/usr/bin/env python3
"""Extend the existing simulation latch with accepted NDT measurement freshness.

EKF prediction output timestamps alone cannot detect lost absolute localization.
No vehicle controls are published here; the existing pinned gate brakes.
"""
import importlib.util
from pathlib import Path
import time
from ament_index_python.packages import get_package_prefix
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

path = Path(get_package_prefix('atlas_closed_loop_demo')) / 'lib/atlas_closed_loop_demo/feedback_watchdog.py'
spec = importlib.util.spec_from_file_location('atlas_existing_watchdog', path)
existing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(existing)


class LocalizationWatchdog(existing.Watchdog):
    def __init__(self):
        super().__init__()
        self.create_subscription(PoseWithCovarianceStamped, 'ndt/ndt_pose_with_covariance',
                                 lambda msg: self.receive('ndt', msg), 10)

    def stale(self):
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
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=.02)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
