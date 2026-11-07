#!/usr/bin/env python3
"""Publish a lightweight visualization-only body marker from EKF odometry."""

import math
import time
from collections import Counter

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def _rotate(q, vector):
    """Rotate a vector by quaternion q without importing a transform stack."""
    x, y, z = vector
    qx, qy, qz, qw = q
    # q * [v, 0] * conjugate(q)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def make_ego_markers(odom):
    """Build a map-frame placeholder UGV body and a forward heading arrow.

    Geometry is for visualization only: 3.1 m long, 1.5 m wide and 0.9 m high.
    The body center is placed 0.75 m forward of base_link to resemble the
    existing sample vehicle description's rear-axle reference convention.
    """
    if marker_rejection_reason(odom) is not None:
        return None
    pose = odom.pose.pose
    p = pose.position
    qmsg = pose.orientation
    q = (qmsg.x, qmsg.y, qmsg.z, qmsg.w)
    values = (p.x, p.y, p.z, *q)
    if not all(math.isfinite(v) for v in values):
        return None
    qnorm = math.sqrt(sum(v * v for v in q))
    if qnorm < 1e-9:
        return None
    q = tuple(v / qnorm for v in q)

    def world_point(local):
        rotated = _rotate(q, local)
        return Point(x=p.x + rotated[0], y=p.y + rotated[1], z=p.z + rotated[2])

    stamp = odom.header.stamp
    body = Marker()
    body.header.frame_id = 'map'
    body.header.stamp = stamp
    body.ns = 'atlas_ego_visualization'
    body.id = 0
    body.type = Marker.CUBE
    body.action = Marker.ADD
    body.pose.position = world_point((0.75, 0.0, 0.48))
    body.pose.orientation.x, body.pose.orientation.y, body.pose.orientation.z, body.pose.orientation.w = q
    body.scale.x, body.scale.y, body.scale.z = 3.1, 1.5, 0.9
    body.color.r, body.color.g, body.color.b, body.color.a = 0.05, 0.72, 0.95, 0.96
    body.lifetime.sec = 0
    body.lifetime.nanosec = 500_000_000

    arrow = Marker()
    arrow.header.frame_id = 'map'
    arrow.header.stamp = stamp
    arrow.ns = 'atlas_ego_visualization'
    arrow.id = 1
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.points = [world_point((1.35, 0.0, 0.98)), world_point((2.55, 0.0, 0.98))]
    arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.16, 0.36, 0.42
    arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.32, 0.02, 1.0
    arrow.lifetime.sec = 0
    arrow.lifetime.nanosec = 500_000_000
    return MarkerArray(markers=[body, arrow])


def marker_rejection_reason(odom):
    """Describe pose/frame data this map-frame display cannot use."""
    if odom.header.frame_id != 'map' or odom.child_frame_id != 'base_link':
        return f"frame mismatch header={odom.header.frame_id!r} child={odom.child_frame_id!r}; expected map/base_link"
    pose = odom.pose.pose
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w)
    if not all(math.isfinite(v) for v in values):
        return 'nonfinite EKF pose'
    if math.sqrt(sum(v * v for v in values[3:])) < 1e-9:
        return 'zero-norm EKF orientation quaternion'
    return None


class EgoPoseMarker(Node):
    def __init__(self):
        super().__init__('atlas_ego_pose_marker')
        self.publisher = self.create_publisher(MarkerArray, '/atlas_loc/visualization/ego_vehicle', 10)
        self.subscription = self.create_subscription(
            Odometry, '/atlas_loc/ekf/ekf_odom', self.on_odom, 10)
        self.started_monotonic = time.monotonic()
        self.last_receipt_monotonic = None
        self.received_count = 0
        self.published_count = 0
        self.rejected_counts = Counter()
        self.logged_reasons = set()
        self.create_timer(5.0, self.report_status)
        self.get_logger().info(
            'Started; waiting for /atlas_loc/ekf/ekf_odom. '
            'Markers use EKF map/base_link pose; RViz fixed frame should be map. '
            'No TF lookup or simulator ground truth is used.')

    def on_odom(self, msg):
        self.received_count += 1
        self.last_receipt_monotonic = time.monotonic()
        if self.received_count == 1:
            self.get_logger().info(
                f'Received first EKF odometry: frame={msg.header.frame_id!r}, '
                f'child={msg.child_frame_id!r}, stamp={msg.header.stamp.sec}.'
                f'{msg.header.stamp.nanosec:09d}')
        reason = marker_rejection_reason(msg)
        if reason is not None:
            self.rejected_counts[reason.split(';')[0]] += 1
            if reason not in self.logged_reasons:
                self.get_logger().warning(f'Cannot display EKF pose: {reason}')
                self.logged_reasons.add(reason)
            return
        markers = make_ego_markers(msg)
        if markers is None:
            return
        self.publisher.publish(markers)
        self.published_count += 1
        if self.published_count == 1:
            self.get_logger().info(
                'Published first EKF ego MarkerArray: 2 markers in map '
                '(body + forward arrow), 0.5 s lifetime.')

    def report_status(self):
        if self.received_count == 0:
            self.get_logger().warning(
                'Still waiting for EKF odometry: received=0, '
                f'marker_subscribers={self.publisher.get_subscription_count()}. '
                'Check EKF activation, ROS_DOMAIN_ID, and /atlas_loc/ekf/ekf_odom.')
            return
        receipt_age = time.monotonic() - self.last_receipt_monotonic
        self.get_logger().info(
            f'EKF marker status: received={self.received_count}, '
            f'published={self.published_count}, rejected={sum(self.rejected_counts.values())}, '
            f'last_receipt_age_sec={receipt_age:.3f}, '
            f'marker_subscribers={self.publisher.get_subscription_count()}')


def main(args=None):
    rclpy.init(args=args)
    node = EgoPoseMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
