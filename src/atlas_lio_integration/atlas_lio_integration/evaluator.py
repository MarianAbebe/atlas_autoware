"""Evaluator-only 3D trajectory comparison; never imported by FAST-LIO."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def valid_stamp(stamp):
    return (isinstance(stamp.sec, int) and isinstance(stamp.nanosec, int)
            and stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000
            and (stamp.sec > 0 or stamp.nanosec > 0))


def _normalize(q):
    norm = math.sqrt(sum(float(v) * float(v) for v in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid quaternion')
    return tuple(float(v) / norm for v in q)


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _normalize((aw*bx + ax*bw + ay*bz - az*by,
                       aw*by - ax*bz + ay*bw + az*bx,
                       aw*bz + ax*by - ay*bx + az*bw,
                       aw*bw - ax*bx - ay*by - az*bz))


def _inverse(q):
    x, y, z, w = _normalize(q)
    return -x, -y, -z, w


def _rotate(q, vector):
    x, y, z, w = _normalize(q)
    vx, vy, vz = vector
    # Equivalent to q * (v, 0) * inverse(q), without temporary quaternions.
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx + y*tz-z*ty,
            vy+w*ty + z*tx-x*tz,
            vz+w*tz + x*ty-y*tx)


def _yaw(q):
    x, y, z, w = _normalize(q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def yaw_from_quaternion(q):
    """Compatibility helper accepting a ROS Quaternion object."""
    return _yaw((q.x, q.y, q.z, q.w))


def _slerp(a, b, ratio):
    a, b = _normalize(a), _normalize(b)
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-v for v in b)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return _normalize(tuple(x + ratio*(y-x) for x, y in zip(a, b)))
    theta = math.acos(dot)
    scale_a = math.sin((1.0-ratio)*theta) / math.sin(theta)
    scale_b = math.sin(ratio*theta) / math.sin(theta)
    return _normalize(tuple(scale_a*x + scale_b*y for x, y in zip(a, b)))


def interpolate_truth(samples, timestamp, max_bracket_sec=0.11):
    """Interpolate (stamp, XYZ tuple, XYZW tuple) without extrapolation."""
    if len(samples) < 2 or not math.isfinite(timestamp):
        return None
    # Accept the pre-SE(3) in-memory test fixture format (t, x, y, z, yaw).
    samples = [((row[0], (row[1], row[2], row[3]),
                 (0.0, 0.0, math.sin(row[4]/2.0), math.cos(row[4]/2.0)))
                if len(row) == 5 and isinstance(row[1], (int, float)) else row)
               for row in samples]
    for left, right in zip(samples, samples[1:]):
        if left[0] <= timestamp <= right[0]:
            span = right[0] - left[0]
            if span <= 0.0 or span > max_bracket_sec:
                return None
            ratio = (timestamp-left[0]) / span
            position = tuple(a + ratio*(b-a) for a, b in zip(left[1], right[1]))
            try:
                orientation = _slerp(left[2], right[2], ratio)
            except ValueError:
                return None
            return position, orientation
    return None


def _rmse(values):
    return math.sqrt(sum(value*value for value in values) / len(values)) if values else None


def _rotation_error(a, b):
    relative = _multiply(_inverse(a), b)
    return 2.0 * math.acos(min(1.0, abs(relative[3])))


class Evaluator(Node):
    def __init__(self):
        super().__init__('atlas_lio_evaluator')
        self.truth_frame = self.declare_parameter('ground_truth_frame', 'map').value
        self.estimate_frame = self.declare_parameter('estimator_frame', 'odom').value
        self.estimate_child_frame = self.declare_parameter('estimator_child_frame', 'base_link').value
        self.max_bracket = float(self.declare_parameter('max_ground_truth_bracket_sec', 0.11).value)
        self.duration = float(self.declare_parameter('duration_sec', 15.0).value)
        if not math.isfinite(self.max_bracket) or self.max_bracket <= 0.0:
            raise ValueError('max_ground_truth_bracket_sec must be finite and positive')
        if not math.isfinite(self.duration) or self.duration <= 0.0:
            raise ValueError('duration_sec must be finite and positive')
        self.truth = []
        self.samples = []
        self.errors = []
        self.odom_times = []
        self.rejected_matches = 0
        self.valid_unique_estimator_samples = 0
        self.invalid_estimates = 0
        self.invalid_truth = 0
        self.frame_rejections = 0
        self.timestamp_rejections = {'invalid': 0, 'duplicate': 0, 'regressing': 0}
        self.alignment = None
        self.last_truth_stamp = None
        self.last_estimate_stamp = None
        self.create_subscription(PoseStamped, '/atlas/sensors/evaluation/ground_truth', self.gt, 100)
        self.create_subscription(Odometry, '/atlas/lio/odometry', self.odom, 100)

    def gt(self, msg):
        timestamp = stamp_seconds(msg.header.stamp)
        if (not msg.header.frame_id or msg.header.frame_id != self.truth_frame
                or not valid_stamp(msg.header.stamp) or not math.isfinite(timestamp)):
            self.invalid_truth += 1
            return
        if self.last_truth_stamp is not None and timestamp <= self.last_truth_stamp:
            self.invalid_truth += 1
            return
        position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        try:
            orientation = _normalize((msg.pose.orientation.x, msg.pose.orientation.y,
                                      msg.pose.orientation.z, msg.pose.orientation.w))
        except ValueError:
            self.invalid_truth += 1
            return
        if not all(math.isfinite(value) for value in position):
            self.invalid_truth += 1
            return
        self.truth.append((timestamp, position, orientation))
        self.truth = self.truth[-1000:]
        self.last_truth_stamp = timestamp

    def odom(self, msg):
        if not hasattr(self, 'timestamp_rejections'):
            self.timestamp_rejections = {'invalid': 0, 'duplicate': 0, 'regressing': 0}
        timestamp = stamp_seconds(msg.header.stamp)
        if (msg.header.frame_id != self.estimate_frame
                or msg.child_frame_id != getattr(self, 'estimate_child_frame', 'base_link')):
            self.frame_rejections += 1
            return
        if not valid_stamp(msg.header.stamp) or not math.isfinite(timestamp):
            self.timestamp_rejections['invalid'] += 1
            return
        last_estimate_stamp = getattr(self, 'last_estimate_stamp', None)
        if last_estimate_stamp is not None and timestamp <= last_estimate_stamp:
            reason = 'duplicate' if timestamp == last_estimate_stamp else 'regressing'
            self.timestamp_rejections[reason] += 1
            return
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        try:
            orientation = _normalize((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        except ValueError:
            self.invalid_estimates += 1
            return
        if not all(math.isfinite(value) for value in position):
            self.invalid_estimates += 1
            return
        self.last_estimate_stamp = timestamp
        if not hasattr(self, 'valid_unique_estimator_samples'):
            self.valid_unique_estimator_samples = 0
        self.valid_unique_estimator_samples += 1
        truth = interpolate_truth(self.truth, timestamp, self.max_bracket)
        if truth is None:
            self.rejected_matches += 1
            return
        truth_position, truth_orientation = truth
        if self.alignment is None:
            # One fixed SE(3) transform from the first timestamp-matched pair.
            rotation = _multiply(truth_orientation, _inverse(orientation))
            rotated = _rotate(rotation, position)
            translation = tuple(a-b for a, b in zip(truth_position, rotated))
            self.alignment = (rotation, translation)
        rotation, translation = self.alignment
        aligned_position = tuple(a+b for a, b in zip(_rotate(rotation, position), translation))
        aligned_orientation = _multiply(rotation, orientation)
        ex, ey, ez = (a-b for a, b in zip(aligned_position, truth_position))
        xy_error = math.hypot(ex, ey)
        z_error = abs(ez)
        yaw_error = abs(math.atan2(math.sin(_yaw(aligned_orientation)-_yaw(truth_orientation)),
                                   math.cos(_yaw(aligned_orientation)-_yaw(truth_orientation))))
        errors = (xy_error, z_error, math.sqrt(ex*ex+ey*ey+ez*ez), yaw_error)
        if not all(math.isfinite(value) for value in errors):
            self.invalid_estimates += 1
            return
        self.samples.append((timestamp, aligned_position, aligned_orientation,
                             truth_position, truth_orientation, position, orientation, errors))
        self.errors.append((xy_error, yaw_error))
        self.odom_times.append(timestamp)

    def result(self):
        # Retain legacy unit-fixture construction without changing runtime metrics.
        if not getattr(self, 'samples', []):
            legacy = getattr(self, 'errors', [])
            if not legacy:
                return {'result': 'FAIL', 'reason': 'no valid timestamp matched estimator odometry'}
            xy = _rmse([row[0] for row in legacy])
            yaw = _rmse([row[1] for row in legacy])
            duration = max(self.odom_times)-min(self.odom_times) if len(self.odom_times) > 1 else 0.0
            rate = (len(self.odom_times)-1)/duration if duration > 0.0 else 0.0
            rejected = getattr(self, 'rejected_matches', 0)
            match_rate = len(legacy)/(len(legacy)+rejected) if legacy or rejected else 0.0
            passed = xy <= 0.50 and yaw <= 0.15 and len(legacy) >= 100 and rate > 0.0 and match_rate >= 0.90
            return {'result': 'PASS' if passed else 'FAIL', 'samples': len(legacy),
                    'rejected_matches': rejected, 'match_rate': match_rate,
                    'translation_rmse_m': xy, 'orientation_rmse_rad': yaw, 'output_rate_hz': rate}

        xy = [sample[7][0] for sample in self.samples]
        z = [sample[7][1] for sample in self.samples]
        d3 = [sample[7][2] for sample in self.samples]
        yaw = [sample[7][3] for sample in self.samples]
        relative_translation = []
        relative_rotation = []
        distance = 0.0
        for previous, current in zip(self.samples, self.samples[1:]):
            distance += math.dist(previous[3], current[3])
            est_delta = _rotate(_inverse(previous[2]), tuple(b-a for a, b in zip(previous[1], current[1])))
            truth_delta = _rotate(_inverse(previous[4]), tuple(b-a for a, b in zip(previous[3], current[3])))
            relative_translation.append(math.dist(est_delta, truth_delta))
            est_rotation = _multiply(_inverse(previous[2]), current[2])
            truth_rotation = _multiply(_inverse(previous[4]), current[4])
            relative_rotation.append(_rotation_error(est_rotation, truth_rotation))

        count = len(self.samples)
        duration = self.samples[-1][0]-self.samples[0][0] if count > 1 else 0.0
        rate = (count-1)/duration if duration > 0.0 else None
        timestamp_rejections = getattr(self, 'timestamp_rejections', {'invalid': 0, 'duplicate': 0, 'regressing': 0})
        rejected = (self.rejected_matches + self.invalid_estimates + self.frame_rejections
                    + sum(timestamp_rejections.values()))
        # Match rate is over unique, finite, frame-valid estimator poses. Repeated
        # publication of one state at an identical stamp is counted separately,
        # not misreported as multiple missing localization measurements.
        match_denominator = count + self.rejected_matches
        match_rate = count/match_denominator if match_denominator else 0.0
        endpoint_drift = d3[-1]
        rotation, translation = self.alignment
        absolute_alignment = {'translation_m': list(translation), 'rotation_xyzw': list(rotation)}
        valid = all(math.isfinite(value) for group in (xy, z, d3, yaw, relative_translation, relative_rotation) for value in group)
        # These are the pre-existing acceptance limits; relative metrics are reported, not gate criteria.
        passed = valid and _rmse(xy) <= 0.50 and _rmse(yaw) <= 0.15 and count >= 100 and rate is not None and rate > 0.0 and match_rate >= 0.90
        return {
            'result': 'PASS' if passed else 'FAIL',
            'accuracy_mode': 'fixed_initial_pose_se3_aligned',
            'ground_truth_frame': self.truth_frame,
            'estimator_frame': self.estimate_frame,
            'estimator_child_frame': self.estimate_child_frame,
            'initial_alignment': absolute_alignment,
            'matched_sample_count': count,
            'rejected_sample_count': rejected,
            'valid_unique_estimator_sample_count': getattr(self, 'valid_unique_estimator_samples', count+self.rejected_matches),
            'unmatched_valid_estimator_sample_count': self.rejected_matches,
            'rejected_samples_by_reason': {
                'ground_truth_invalid_or_nonmonotonic': self.invalid_truth,
                'estimator_invalid_pose': self.invalid_estimates,
                'frame_mismatch': self.frame_rejections,
                'outside_or_unbracketed_ground_truth': self.rejected_matches,
                'estimator_invalid_timestamp': timestamp_rejections['invalid'],
                'estimator_duplicate_timestamp': timestamp_rejections['duplicate'],
                'estimator_regressing_timestamp': timestamp_rejections['regressing'],
                'estimator_invalid_pose': self.invalid_estimates,
                'estimator_frame_mismatch': self.frame_rejections,
            },
            'match_rate': match_rate,
            'xy_rmse_m': _rmse(xy), 'xy_max_error_m': max(xy),
            'z_rmse_m': _rmse(z), 'z_max_error_m': max(z),
            '3d_rmse_m': _rmse(d3), '3d_max_error_m': max(d3),
            'yaw_rmse_rad': _rmse(yaw), 'yaw_max_error_rad': max(yaw),
            'relative_translation_error_rmse_m': _rmse(relative_translation),
            'relative_translation_error_max_m': max(relative_translation) if relative_translation else None,
            'relative_rotation_error_rmse_rad': _rmse(relative_rotation),
            'relative_rotation_error_max_rad': max(relative_rotation) if relative_rotation else None,
            'trajectory_duration_sec': duration,
            'distance_traveled_m': distance,
            'endpoint_aligned_position_error_m': endpoint_drift,
            'output_rate_hz': rate,
            'acceptance_criteria': {'xy_rmse_m_max': 0.50, 'yaw_rmse_rad_max': 0.15,
                                    'matched_sample_count_min': 100, 'output_rate_hz_min_exclusive': 0.0,
                                    'match_rate_min': 0.90},
        }


def main():
    rclpy.init()
    node = Evaluator()
    end = time.monotonic() + node.duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(json.dumps(node.result(), allow_nan=False))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
