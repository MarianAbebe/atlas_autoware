"""Deterministic 2D ray-cast sensor for the synthetic occupancy-map world."""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

from .world import occupied, rectangles


class SyntheticLaser(Node):
    def __init__(self):
        super().__init__('synthetic_laser')
        self.pose = None
        self.pose_stamp = None
        self.scenario = self.declare_parameter('scenario', 'open').value
        self.odom_topic = self.declare_parameter('odom_topic', '/atlas_nav2/odom').value
        self.scan_topic = self.declare_parameter('scan_topic', '/atlas_nav2/scan').value
        self.output_enabled = True
        rectangles(self.scenario)
        self.publisher = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.create_service(SetBool, '~/set_output_enabled', self.on_output_enabled)
        self.timer = self.create_timer(0.10, self.tick)

    def on_output_enabled(self, request, response):
        self.output_enabled = bool(request.data)
        response.success = True
        response.message = ('synthetic scan publishing enabled' if request.data
                            else 'synthetic scan publishing paused')
        return response

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        # The scan is ray-cast from this odometry pose. Keep its source time so
        # TF lookup uses the pose time instead of stamping stale geometry as now.
        self.pose_stamp = msg.header.stamp

    def tick(self):
        if not self.output_enabled or self.pose is None or self.pose_stamp is None:
            return
        scan = LaserScan()
        scan.header.stamp = self.pose_stamp
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
