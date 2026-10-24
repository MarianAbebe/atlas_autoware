"""Deterministic 2D ray-cast sensor for the synthetic occupancy-map world."""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .world import occupied, rectangles


class SyntheticLaser(Node):
    def __init__(self):
        super().__init__('synthetic_laser')
        self.pose = None
        self.scenario = self.declare_parameter('scenario', 'open').value
        rectangles(self.scenario)
        self.publisher = self.create_publisher(LaserScan, '/atlas_nav2/scan', 10)
        self.create_subscription(Odometry, '/atlas_nav2/odom', self.on_odom, 20)
        self.timer = self.create_timer(0.10, self.tick)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def tick(self):
        if self.pose is None:
            return
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -2.35619449
        scan.angle_max = 2.35619449
        scan.angle_increment = math.radians(1.0)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.08
        scan.range_max = 8.0
        count = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
        x, y, yaw = self.pose
        ranges = []
        for index in range(count):
            angle = yaw + scan.angle_min + index * scan.angle_increment
            distance = scan.range_min
            while distance < scan.range_max:
                sx = x + 0.20 * math.cos(yaw) + distance * math.cos(angle)
                sy = y + 0.20 * math.sin(yaw) + distance * math.sin(angle)
                if occupied(sx, sy, self.scenario):
                    break
                distance += 0.04
            ranges.append(min(distance, scan.range_max))
        scan.ranges = ranges
        self.publisher.publish(scan)


def main():
    rclpy.init()
    node = SyntheticLaser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
