"""Ideal no-slip four-wheel skid-steer kinematic simulator."""
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .kinematics import wheels_to_twist
from .wheel_mixer import WHEEL_NAMES
from .world import clearance, rectangles


class SkidSteerSimulator(Node):
    def __init__(self):
        super().__init__('skid_steer_simulator')
        self.radius = self.declare_parameter('wheel_radius', 0.15).value
        self.track = self.declare_parameter('track_width', 0.60).value
        self.timeout = self.declare_parameter('target_timeout_sec', 0.30).value
        self.rate = self.declare_parameter('update_rate_hz', 50.0).value
        self.scenario = self.declare_parameter('scenario', 'open').value
        self.robot_radius = self.declare_parameter('robot_radius', 0.32).value
        rectangles(self.scenario)
        wheels_to_twist(0.0, 0.0, self.track, self.radius)
        self.x = self.y = self.yaw = 0.0
        self.positions = [0.0] * 4
        self.targets = [0.0] * 4
        self.last_target = None
        self.last_update = time.monotonic()
        self.odom_pub = self.create_publisher(Odometry, '/atlas_nav2/odom', 20)
        self.wheels_pub = self.create_publisher(JointState, '/atlas_nav2/wheel_states', 20)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.create_subscription(JointState, '/atlas_nav2/wheel_speed_targets', self.on_targets, 10)
        self.publish_static_transforms()
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    def publish_static_transforms(self):
        now = self.get_clock().now().to_msg()
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = now
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.rotation.w = 1.0
        base_to_laser = TransformStamped()
        base_to_laser.header.stamp = now
        base_to_laser.header.frame_id = 'base_link'
        base_to_laser.child_frame_id = 'laser'
        base_to_laser.transform.translation.x = 0.20
        base_to_laser.transform.translation.z = 0.25
        base_to_laser.transform.rotation.w = 1.0
        self.static_tf.sendTransform([map_to_odom, base_to_laser])

    def on_targets(self, msg):
        if msg.name != WHEEL_NAMES or len(msg.velocity) != 4:
            self.get_logger().error('Rejected wheel target with invalid names or size')
            return
        if not all(math.isfinite(value) for value in msg.velocity):
            self.get_logger().error('Rejected non-finite wheel target')
            return
        self.targets = list(msg.velocity)
        self.last_target = time.monotonic()

    def tick(self):
        monotonic = time.monotonic()
        dt = min(0.05, max(0.0, monotonic - self.last_update))
        self.last_update = monotonic
        velocities = self.targets if self.last_target is not None and monotonic - self.last_target <= self.timeout else [0.0] * 4
        left = (velocities[0] + velocities[1]) / 2.0
        right = (velocities[2] + velocities[3]) / 2.0
        linear, angular = wheels_to_twist(left, right, self.track, self.radius)
        proposed_x = self.x + linear * math.cos(self.yaw) * dt
        proposed_y = self.y + linear * math.sin(self.yaw) * dt
        if clearance(proposed_x, proposed_y, self.robot_radius, self.scenario) < 0.0:
            # Ideal contact: never integrate a pose through a map obstacle.
            linear = angular = 0.0
        else:
            self.x, self.y = proposed_x, proposed_y
            self.yaw = math.atan2(math.sin(self.yaw + angular * dt), math.cos(self.yaw + angular * dt))
        self.positions = [p + w * dt for p, w in zip(self.positions, velocities)]
        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        odom.pose.covariance[0] = odom.pose.covariance[7] = 1e-6
        odom.pose.covariance[35] = 1e-6
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.sendTransform(transform)

        wheels = JointState()
        wheels.header.stamp = stamp
        wheels.name = WHEEL_NAMES
        wheels.position = self.positions
        wheels.velocity = velocities
        self.wheels_pub.publish(wheels)


def main():
    rclpy.init()
    node = SkidSteerSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.targets = [0.0] * 4
        node.destroy_node()
        rclpy.try_shutdown()
