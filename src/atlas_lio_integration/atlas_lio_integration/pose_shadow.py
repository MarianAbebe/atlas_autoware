"""Pose-only shadowing for FAST-LIO odometry with unusable twist fields."""
import copy
import math

from geometry_msgs.msg import PoseStamped


REJECTION_KEYS = (
    'duplicate_timestamp_rejections',
    'regressing_timestamp_rejections',
    'invalid_timestamp_rejections',
    'invalid_position_rejections',
    'invalid_quaternion_rejections',
    'frame_mismatch_rejections',
)


def validate_pose_shadow_parameters(expected_frame_id, expected_child_frame_id,
                                    diagnostics_rate_hz, stale_after_sec,
                                    quaternion_norm_tolerance):
    if not expected_frame_id or not expected_child_frame_id:
        raise ValueError('expected frame IDs must be nonempty')
    if not math.isfinite(diagnostics_rate_hz) or not 0.0 < diagnostics_rate_hz <= 20.0:
        raise ValueError('diagnostics_rate_hz must be finite and in (0, 20]')
    if not math.isfinite(stale_after_sec) or stale_after_sec <= 0.0:
        raise ValueError('diagnostic_stale_after_sec must be finite and positive')
    if (not math.isfinite(quaternion_norm_tolerance)
            or not 0.0 < quaternion_norm_tolerance < 1.0):
        raise ValueError('quaternion_norm_tolerance must be finite and in (0, 1)')


def header_stamp_nanoseconds(msg):
    """Return an exact positive ROS header stamp, or None if invalid/zero."""
    stamp = msg.header.stamp
    if (not isinstance(stamp.sec, int) or not isinstance(stamp.nanosec, int)
            or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
        return None
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    return value if value > 0 else None


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def make_pose_diagnostic_array(stamp, level, message, values):
    """Build one bounded standard diagnostic record."""
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    result = DiagnosticArray()
    result.header.stamp = copy.deepcopy(stamp)
    status = DiagnosticStatus()
    status.name = 'atlas_lio/shadow_pose'
    status.hardware_id = 'synthetic_fastlio2'
    status.level = bytes((int(level),))
    status.message = message
    status.values = [
        KeyValue(key=key, value='unavailable' if value is None else str(value))
        for key, value in values.items()
    ]
    result.status = [status]
    return result


class ShadowPoseFilter:
    """Bounded-state pose validator; never accesses twist or covariance fields."""

    def __init__(self, expected_frame_id='odom', expected_child_frame_id='base_link',
                 stale_after_sec=0.3, quaternion_norm_tolerance=0.01):
        validate_pose_shadow_parameters(expected_frame_id, expected_child_frame_id,
                                        2.0, stale_after_sec,
                                        quaternion_norm_tolerance)
        self.expected_frame_id = expected_frame_id
        self.expected_child_frame_id = expected_child_frame_id
        self.stale_after_sec = float(stale_after_sec)
        self.quaternion_norm_tolerance = float(quaternion_norm_tolerance)
        self.total_received_messages = 0
        self.accepted_unique_pose_samples = 0
        self.rejections = {key: 0 for key in REJECTION_KEYS}
        self.last_message_receipt_monotonic = None
        self.last_unique_receipt_monotonic = None
        self.first_unique_stamp_ns = None
        self.last_unique_stamp_ns = None
        self.max_unique_header_gap_sec = 0.0

    def _reject(self, key):
        self.rejections[key] += 1
        return None, key

    def ingest(self, msg, receipt_monotonic_sec):
        """Return (PoseStamped deep copy or None, rejection reason)."""
        self.total_received_messages += 1
        self.last_message_receipt_monotonic = float(receipt_monotonic_sec)

        stamp_ns = header_stamp_nanoseconds(msg)
        if stamp_ns is None:
            return self._reject('invalid_timestamp_rejections')
        if self.last_unique_stamp_ns is not None:
            if stamp_ns == self.last_unique_stamp_ns:
                return self._reject('duplicate_timestamp_rejections')
            if stamp_ns < self.last_unique_stamp_ns:
                return self._reject('regressing_timestamp_rejections')

        if (msg.header.frame_id != self.expected_frame_id
                or msg.child_frame_id != self.expected_child_frame_id):
            return self._reject('frame_mismatch_rejections')

        position = msg.pose.pose.position
        position_values = (position.x, position.y, position.z)
        if not _finite(position_values):
            return self._reject('invalid_position_rejections')

        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        if not _finite(quaternion):
            return self._reject('invalid_quaternion_rejections')
        quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
        if (not math.isfinite(quaternion_norm) or quaternion_norm == 0.0
                or abs(quaternion_norm - 1.0) > self.quaternion_norm_tolerance):
            return self._reject('invalid_quaternion_rejections')

        if self.last_unique_stamp_ns is None:
            self.first_unique_stamp_ns = stamp_ns
        else:
            gap_sec = (stamp_ns - self.last_unique_stamp_ns) * 1e-9
            self.max_unique_header_gap_sec = max(self.max_unique_header_gap_sec, gap_sec)
        self.last_unique_stamp_ns = stamp_ns
        self.last_unique_receipt_monotonic = float(receipt_monotonic_sec)
        self.accepted_unique_pose_samples += 1

        output = PoseStamped()
        output.header = copy.deepcopy(msg.header)
        output.pose = copy.deepcopy(msg.pose.pose)
        return output, None

    def diagnostic_values(self, now_monotonic_sec):
        """Return scalar counters and receipt ages; ROS epoch is never mixed in."""
        message_age = (
            None if self.last_message_receipt_monotonic is None else
            max(0.0, now_monotonic_sec - self.last_message_receipt_monotonic)
        )
        unique_age = (
            None if self.last_unique_receipt_monotonic is None else
            max(0.0, now_monotonic_sec - self.last_unique_receipt_monotonic)
        )
        duration_sec = (
            0.0 if self.first_unique_stamp_ns is None or self.last_unique_stamp_ns is None
            else (self.last_unique_stamp_ns - self.first_unique_stamp_ns) * 1e-9
        )
        rate_hz = (
            (self.accepted_unique_pose_samples - 1) / duration_sec
            if self.accepted_unique_pose_samples > 1 and duration_sec > 0.0 else 0.0
        )

        if self.accepted_unique_pose_samples == 0:
            level, message = 2, 'no accepted unique estimator pose'
        elif unique_age is None or unique_age > self.stale_after_sec:
            level, message = 2, 'unique estimator pose is stale'
        elif any(self.rejections.values()):
            level, message = 1, 'invalid or duplicate pose inputs were filtered'
        else:
            level, message = 0, 'healthy'

        values = {
            'total_received_messages': self.total_received_messages,
            'accepted_unique_pose_samples': self.accepted_unique_pose_samples,
            **self.rejections,
            'unique_pose_publication_rate_hz': rate_hz,
            'max_unique_header_gap_sec': self.max_unique_header_gap_sec,
            'latest_raw_message_receipt_age_sec': message_age,
            'latest_unique_state_receipt_age_sec': unique_age,
            'receipt_age_clock_domain': 'monotonic process clock',
            'status_message': message,
        }
        return level, message, values


def main():
    import time

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class PoseShadowAdapter(Node):
        def __init__(self):
            super().__init__('atlas_lio_shadow_pose')
            param = lambda name, default: self.declare_parameter(name, default).value
            input_topic = param('input_topic', '/atlas/lio/odometry')
            output_topic = param('output_topic', '/atlas/lio/shadow/pose')
            diagnostics_topic = param(
                'diagnostics_topic', '/atlas/lio/shadow/pose_diagnostics'
            )
            expected_frame = param('expected_frame_id', 'odom')
            expected_child = param('expected_child_frame_id', 'base_link')
            diagnostics_rate = float(param('diagnostics_rate_hz', 2.0))
            stale_after = float(param('diagnostic_stale_after_sec', 0.3))
            quaternion_tolerance = float(param('quaternion_norm_tolerance', 0.01))
            validate_pose_shadow_parameters(
                expected_frame, expected_child, diagnostics_rate,
                stale_after, quaternion_tolerance
            )
            self.filter = ShadowPoseFilter(
                expected_frame, expected_child, stale_after, quaternion_tolerance
            )
            self.output = self.create_publisher(PoseStamped, output_topic, 10)
            self.diagnostics = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)
            self.create_subscription(Odometry, input_topic, self.on_odometry, 10)
            self.create_timer(1.0 / diagnostics_rate, self.publish_diagnostics)
            self.get_logger().info(
                f'pose shadow input={input_topic} output={output_topic} '
                f'diagnostics={diagnostics_topic} expected={expected_frame}->{expected_child} '
                f'quaternion_norm_tolerance={quaternion_tolerance}'
            )

        def on_odometry(self, msg):
            accepted, _reason = self.filter.ingest(msg, time.monotonic())
            if accepted is not None:
                self.output.publish(accepted)

        def publish_diagnostics(self):
            now = self.get_clock().now()
            level, message, values = self.filter.diagnostic_values(time.monotonic())
            self.diagnostics.publish(
                make_pose_diagnostic_array(now.to_msg(), level, message, values)
            )

    rclpy.init()
    node = PoseShadowAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
