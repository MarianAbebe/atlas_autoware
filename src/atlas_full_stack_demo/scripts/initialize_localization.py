#!/usr/bin/env python3
"""Activate and initialize NDT/EKF without engaging or commanding the simulator."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool


def stamp(message_stamp):
    return message_stamp.sec + message_stamp.nanosec * 1e-9


class LocalizationInitializer(Node):
    def __init__(self):
        super().__init__('full_stack_localization_initializer')
        self.scene = {}
        self.safety = {}
        self.speed = None
        self.ndt_stamps = []
        self.ekf_stamps = []
        self.create_subscription(
            String, '/atlas_loc/scene_info', self.on_scene,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            String, '/atlas_loc/safety_status', self.on_safety, 10)
        self.create_subscription(
            Odometry, '/atlas_loc/sim/output/odometry', self.on_truth, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/atlas_loc/ndt/ndt_pose_with_covariance',
            lambda msg: self.ndt_stamps.append(stamp(msg.header.stamp)), 10)
        self.create_subscription(
            Odometry, '/atlas_loc/ekf/ekf_odom',
            lambda msg: self.ekf_stamps.append(stamp(msg.header.stamp)), 10)
        self.bootstrap = self.create_publisher(
            PoseWithCovarianceStamped, '/atlas_loc/bootstrap', 10)

    def on_scene(self, msg):
        self.scene.update(json.loads(msg.data))

    def on_safety(self, msg):
        self.safety.update(json.loads(msg.data))

    def on_truth(self, msg):
        self.speed = msg.twist.twist.linear.x

    def wait(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def activate(self, name):
        client = self.create_client(SetBool, f'/atlas_loc/{name}/trigger_node_srv')
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{name.upper()} activation service is unavailable')
        future = client.call_async(SetBool.Request(data=True))
        if not self.wait(future.done, 5.0):
            raise RuntimeError(f'{name.upper()} activation request timed out')
        response = future.result()
        if response is None or not response.success:
            detail = '' if response is None else response.message
            raise RuntimeError(f'{name.upper()} activation failed: {detail}')
        print(f'{name.upper()} activation confirmed (safe if already active).')

    def localization_is_fresh(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        return (bool(self.ndt_stamps) and bool(self.ekf_stamps) and
                0 <= now-self.ndt_stamps[-1] < 0.3 and
                0 <= now-self.ekf_stamps[-1] < 0.3)

    def make_bootstrap(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.scene['origin'][0] + 0.1
        msg.pose.pose.position.y = self.scene['origin'][1] + 0.1
        msg.pose.pose.position.z = self.scene['origin'][2]
        msg.pose.pose.orientation.z = math.sin(self.scene['yaw'] / 2)
        msg.pose.pose.orientation.w = math.cos(self.scene['yaw'] / 2)
        for index, value in [
                (0, 0.25), (7, 0.25), (14, 0.0025),
                (21, 0.0006), (28, 0.0006), (35, 0.06)]:
            msg.pose.covariance[index] = value
        return msg


def main():
    rclpy.init()
    node = LocalizationInitializer()
    try:
        if not node.wait(
                lambda: bool(node.scene) and node.speed is not None and bool(node.safety), 15.0):
            raise RuntimeError(
                'Startup data unavailable: require scene_info, simulator odometry, and safety_status')
        if abs(node.speed) > 0.001:
            raise RuntimeError(
                f'Refusing localization initialization while moving at {node.speed:.6f} m/s')
        if node.safety.get('latched'):
            raise RuntimeError(
                'Refusing localization initialization while combined safety watchdog is latched')

        # Avoid clearing the pinned NDT pose buffer when localization is already healthy.
        if node.wait(node.localization_is_fresh, 0.5):
            print('Localization is already active: fresh NDT and EKF outputs observed; no bootstrap sent.')
            return 0

        # This matches manual_start.py. If a node is active but not producing the complete chain,
        # data=True is accepted and the fresh bootstrap below repopulates NDT's pose buffer.
        node.activate('ekf')
        node.activate('ndt')

        if not node.wait(lambda: node.bootstrap.get_subscription_count() > 0, 5.0):
            raise RuntimeError('No subscriber discovered on /atlas_loc/bootstrap')

        bootstrap = node.make_bootstrap()
        bootstrap_stamp = stamp(bootstrap.header.stamp)
        node.bootstrap.publish(bootstrap)
        print(
            'Published one map-frame bootstrap at '
            f'{bootstrap.header.stamp.sec}.{bootstrap.header.stamp.nanosec:09d}; '
            'simulator engagement was not requested.')

        if not node.wait(
                lambda: any(value >= bootstrap_stamp for value in node.ndt_stamps) and
                any(value >= bootstrap_stamp for value in node.ekf_stamps), 20.0):
            ndt = any(value >= bootstrap_stamp for value in node.ndt_stamps)
            ekf = any(value >= bootstrap_stamp for value in node.ekf_stamps)
            raise RuntimeError(
                f'Localization output timeout after bootstrap: NDT={ndt}, EKF={ekf}')

        print('Localization initialized successfully: post-bootstrap NDT and EKF outputs observed.')
        return 0
    except Exception as error:
        print(f'Localization initialization failed: {error}')
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
