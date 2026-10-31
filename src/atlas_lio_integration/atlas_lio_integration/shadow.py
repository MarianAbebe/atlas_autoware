"""Validation and diagnostics for isolated FAST-LIO odometry shadowing."""
import copy
import math

from nav_msgs.msg import Odometry


REJECTION_KEYS = (
    'duplicate_timestamp_rejections',
    'regressing_timestamp_rejections',
    'invalid_timestamp_rejections',
    'invalid_pose_rejections',
    'invalid_twist_rejections',
    'invalid_covariance_rejections',
    'frame_mismatch_rejections',
)


def validate_shadow_parameters(expected_frame_id, expected_child_frame_id,
                                diagnostics_rate_hz, stale_after_sec):
    if not expected_frame_id or not expected_child_frame_id:
        raise ValueError('expected frame IDs must be nonempty')
    if not math.isfinite(diagnostics_rate_hz) or not 0.0 < diagnostics_rate_hz <= 20.0:
        raise ValueError('diagnostics_rate_hz must be finite and in (0, 20]')
    if not math.isfinite(stale_after_sec) or stale_after_sec <= 0.0:
        raise ValueError('diagnostic_stale_after_sec must be finite and positive')


def header_stamp_seconds(msg):
    stamp = msg.header.stamp
    if (not isinstance(stamp.sec, int) or not isinstance(stamp.nanosec, int)
            or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
        return None
    value = stamp.sec + stamp.nanosec * 1e-9
    return value if math.isfinite(value) and value > 0.0 else None


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def make_diagnostic_array(stamp, level, message, values):
    """Build one fixed-size standard diagnostic record without retaining history."""
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    array = DiagnosticArray()
    array.header.stamp = copy.deepcopy(stamp)
    status = DiagnosticStatus()
    status.name = 'atlas_lio/shadow_odometry'
    status.hardware_id = 'synthetic_fastlio2'
    status.level = bytes((int(level),))
    status.message = message
    status.values = [
        KeyValue(key=key, value='unavailable' if value is None else str(value))
        for key, value in values.items()
    ]
    array.status = [status]
    return array


class ShadowOdometryFilter:
    """Bounded-state validator; stores counters and latest timestamps only."""

    def __init__(self, expected_frame_id='odom', expected_child_frame_id='base_link',
                 stale_after_sec=0.3):
        validate_shadow_parameters(expected_frame_id, expected_child_frame_id, 2.0,
                                   stale_after_sec)
        self.expected_frame_id = expected_frame_id
        self.expected_child_frame_id = expected_child_frame_id
        self.stale_after_sec = float(stale_after_sec)
        self.total_received_messages = 0
        self.accepted_unique_messages = 0
        self.rejections = {key: 0 for key in REJECTION_KEYS}
        self.last_message_receipt_monotonic = None
        self.last_unique_receipt_monotonic = None
        self.last_message_receipt_gap_sec = None
        self.max_message_receipt_gap_sec = 0.0
        self.last_unique_receipt_gap_sec = None
        self.max_unique_receipt_gap_sec = 0.0
        self.first_unique_stamp_sec = None
        self.last_unique_stamp_sec = None
        self.max_unique_timestamp_gap_sec = 0.0

    def _reject(self, key):
        self.rejections[key] += 1
        return None, key

    def ingest(self, msg, receipt_monotonic_sec):
        """Return (deep-copied accepted message or None, rejection reason)."""
        self.total_received_messages += 1
        if self.last_message_receipt_monotonic is not None:
            self.last_message_receipt_gap_sec = max(
                0.0, receipt_monotonic_sec - self.last_message_receipt_monotonic
            )
            self.max_message_receipt_gap_sec = max(
                self.max_message_receipt_gap_sec, self.last_message_receipt_gap_sec
            )
        self.last_message_receipt_monotonic = float(receipt_monotonic_sec)

        stamp = header_stamp_seconds(msg)
        if stamp is None:
            return self._reject('invalid_timestamp_rejections')
        if self.last_unique_stamp_sec is not None:
            if stamp == self.last_unique_stamp_sec:
                return self._reject('duplicate_timestamp_rejections')
            if stamp < self.last_unique_stamp_sec:
                return self._reject('regressing_timestamp_rejections')
        if (msg.header.frame_id != self.expected_frame_id
                or msg.child_frame_id != self.expected_child_frame_id):
            return self._reject('frame_mismatch_rejections')

        pose = msg.pose.pose
        position = (pose.position.x, pose.position.y, pose.position.z)
        quaternion = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
        if not _finite(position) or not _finite(quaternion):
            return self._reject('invalid_pose_rejections')
        quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
        if not math.isfinite(quaternion_norm) or abs(quaternion_norm - 1.0) > 0.01:
            return self._reject('invalid_pose_rejections')

        twist = msg.twist.twist
        twist_values = (twist.linear.x, twist.linear.y, twist.linear.z,
                        twist.angular.x, twist.angular.y, twist.angular.z)
        if not _finite(twist_values):
            return self._reject('invalid_twist_rejections')
        if (len(msg.pose.covariance) != 36 or len(msg.twist.covariance) != 36
                or not _finite(msg.pose.covariance)
                or not _finite(msg.twist.covariance)):
            return self._reject('invalid_covariance_rejections')

        if self.last_unique_stamp_sec is None:
            self.first_unique_stamp_sec = stamp
        else:
            self.max_unique_timestamp_gap_sec = max(
                self.max_unique_timestamp_gap_sec, stamp - self.last_unique_stamp_sec
            )
            self.last_unique_receipt_gap_sec = max(
                0.0, receipt_monotonic_sec - self.last_unique_receipt_monotonic
            )
            self.max_unique_receipt_gap_sec = max(
                self.max_unique_receipt_gap_sec, self.last_unique_receipt_gap_sec
            )
        self.last_unique_stamp_sec = stamp
        self.last_unique_receipt_monotonic = float(receipt_monotonic_sec)
        self.accepted_unique_messages += 1
        return copy.deepcopy(msg), None

    def diagnostic_values(self, now_monotonic_sec, now_ros_sec):
        """Return bounded scalar diagnostics; monotonic and ROS clocks stay separate."""
        receipt_age = (
            None if self.last_message_receipt_monotonic is None else
            max(0.0, now_monotonic_sec - self.last_message_receipt_monotonic)
        )
        unique_receipt_age = (
            None if self.last_unique_receipt_monotonic is None else
            max(0.0, now_monotonic_sec - self.last_unique_receipt_monotonic)
        )
        header_age = None
        if (self.last_unique_stamp_sec is not None and math.isfinite(now_ros_sec)
                and now_ros_sec > 0.0):
            header_age = now_ros_sec - self.last_unique_stamp_sec
        duration = (
            0.0 if self.first_unique_stamp_sec is None or self.last_unique_stamp_sec is None
            else self.last_unique_stamp_sec - self.first_unique_stamp_sec
        )
        output_rate = (
            (self.accepted_unique_messages - 1) / duration
            if self.accepted_unique_messages > 1 and duration > 0.0 else 0.0
        )
        if self.accepted_unique_messages == 0:
            level, message = 2, 'no accepted unique estimator state'
        elif header_age is None:
            level, message = 1, 'ROS clock not initialized for header-age check'
        elif header_age < 0.0:
            level, message = 1, 'latest estimator stamp is ahead of ROS clock'
        elif (header_age > self.stale_after_sec or unique_receipt_age is None
              or unique_receipt_age > self.stale_after_sec):
            level, message = 2, 'unique estimator state is stale'
        elif any(self.rejections.values()):
            level, message = 1, 'invalid or duplicate inputs were filtered'
        else:
            level, message = 0, 'healthy'
        values = {
            'total_received_messages': self.total_received_messages,
            'accepted_unique_messages': self.accepted_unique_messages,
            **self.rejections,
            'unique_output_rate_hz': output_rate,
            'max_unique_timestamp_gap_sec': self.max_unique_timestamp_gap_sec,
            'last_message_receipt_gap_sec': self.last_message_receipt_gap_sec,
            'max_message_receipt_gap_sec': self.max_message_receipt_gap_sec,
            'last_unique_receipt_gap_sec': self.last_unique_receipt_gap_sec,
            'max_unique_receipt_gap_sec': self.max_unique_receipt_gap_sec,
            'message_receipt_age_sec': receipt_age,
            'unique_state_receipt_age_sec': unique_receipt_age,
            'latest_estimator_header_age_sec': header_age,
            'freshness_clock_domains': 'receipt=monotonic; header=ROS clock',
            'status_message': message,
        }
        return level, message, values


def main():
    import math
    import time

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class ShadowAdapter(Node):
        def __init__(self):
            super().__init__('atlas_lio_shadow_odometry')
            param = lambda name, default: self.declare_parameter(name, default).value
            input_topic = param('input_topic', '/atlas/lio/odometry')
            output_topic = param('output_topic', '/atlas/lio/shadow/odometry')
            diagnostics_topic = param('diagnostics_topic', '/atlas/lio/shadow/diagnostics')
            expected_frame = param('expected_frame_id', 'odom')
            expected_child = param('expected_child_frame_id', 'base_link')
            diagnostics_rate = float(param('diagnostics_rate_hz', 2.0))
            stale_after = float(param('diagnostic_stale_after_sec', 0.3))
            validate_shadow_parameters(expected_frame, expected_child,
                                       diagnostics_rate, stale_after)
            self.filter = ShadowOdometryFilter(expected_frame, expected_child, stale_after)
            self.output = self.create_publisher(Odometry, output_topic, 10)
            self.diagnostics = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)
            self.create_subscription(Odometry, input_topic, self.on_odometry, 10)
            self.create_timer(1.0 / diagnostics_rate, self.publish_diagnostics)
            self.get_logger().info(
                f'shadow adapter input={input_topic} output={output_topic} '
                f'diagnostics={diagnostics_topic} expected={expected_frame}->{expected_child}'
            )

        def on_odometry(self, msg):
            accepted, _reason = self.filter.ingest(msg, time.monotonic())
            if accepted is not None:
                self.output.publish(accepted)

        def publish_diagnostics(self):
            now_ros = self.get_clock().now()
            now_ros_sec = now_ros.nanoseconds * 1e-9 if now_ros.nanoseconds > 0 else math.nan
            level, message, values = self.filter.diagnostic_values(time.monotonic(), now_ros_sec)
            self.diagnostics.publish(
                make_diagnostic_array(now_ros.to_msg(), level, message, values)
            )

    rclpy.init()
    node = ShadowAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
