#!/usr/bin/env python3
"""Synthetic cloud fixture only: never publishes trajectories or controls."""
import copy
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointCloud2
from autoware_perception_msgs.msg import PredictedObjects


class Source(Node):
    def __init__(self):
        super().__init__('obstacle_source')
        self.cloud_pub = self.create_publisher(PointCloud2, 'obstacles/pointcloud', qos_profile_sensor_data)
        self.objects_pub = self.create_publisher(PredictedObjects, 'obstacles/predicted_objects', 10)
        self.status_pub = self.create_publisher(String, 'obstacles/source_status', 10)
        self.create_subscription(String, 'obstacles/inject', self.inject, 10)
        self.create_subscription(String, 'test_shutdown', self.shutdown, 10)
        self.shutdown_requested = False
        self.mode = 'clear'
        self.anchor = None
        self.last_cloud = None
        self.frozen_cloud = None
        self.create_timer(0.05, self.tick)

    def shutdown(self, _):
        self.shutdown_requested = True

    def inject(self, msg):
        data = json.loads(msg.data)
        mode = data['mode']
        if mode not in ('clear', 'obstacle', 'freeze', 'drop'):
            raise ValueError('Unsupported fixture mode')
        if mode == 'obstacle':
            self.anchor = {k: float(data[k]) for k in ('x', 'y', 'z', 'yaw')}
        if mode == 'freeze':
            self.frozen_cloud = copy.deepcopy(self.last_cloud)
        self.mode = mode

    def tick(self):
        now = self.get_clock().now().to_msg()
        objects = PredictedObjects()
        objects.header = Header(stamp=now, frame_id='map')
        # Required by the plugin even for a pointcloud-only experiment.
        self.objects_pub.publish(objects)
        cloud = None
        if self.mode == 'freeze':
            cloud = self.frozen_cloud
        elif self.mode != 'drop':
            points = []
            if self.mode == 'obstacle' and self.anchor:
                a = self.anchor
                c, s = math.cos(a['yaw']), math.sin(a['yaw'])
                for ix in range(9):
                    for iy in range(7):
                        for iz in range(4):
                            x, y, z = -0.4 + ix * 0.1, -0.3 + iy * 0.1, 0.1 + iz * 0.1
                            points.append((a['x'] + c*x - s*y, a['y'] + s*x + c*y, a['z'] + z))
            cloud = point_cloud2.create_cloud_xyz32(Header(stamp=now, frame_id='map'), points)
        if cloud is not None:
            self.cloud_pub.publish(cloud)
            self.last_cloud = copy.deepcopy(cloud)
        status = String()
        status.data = json.dumps({'stamp': now.sec + now.nanosec*1e-9,
                                  'mode': self.mode, 'anchor': self.anchor,
                                  'published_cloud_stamp': None if cloud is None else
                                  cloud.header.stamp.sec + cloud.header.stamp.nanosec*1e-9,
                                  'points': 0 if cloud is None else cloud.width*cloud.height,
                                  'synthetic': True, 'predicted_objects_count': 0})
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = Source()
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
