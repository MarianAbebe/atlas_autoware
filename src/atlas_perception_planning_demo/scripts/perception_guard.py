#!/usr/bin/env python3
"""Simulation-only authorization latch. Pinned gate supplies all fault braking."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import PredictedObjects
from autoware_adapi_v1_msgs.msg import ManualOperatorHeartbeat
from std_msgs.msg import String
from std_srvs.srv import Trigger


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def sample_health(samples, names, ros_now, mono_now):
    return {name: {'source_stamp': samples[name][0] if name in samples else None,
                   'stamp_age_sec': ros_now-samples[name][0] if name in samples else None,
                   'stamp_age_at_receipt_sec': samples[name][3] if name in samples else None,
                   'receipt_age_sec': mono_now-samples[name][1] if name in samples else None,
                   'receipt_gap_sec': samples[name][2] if name in samples else None}
            for name in names}


class Guard(Node):
    def __init__(self):
        super().__init__('obstacle_guard')
        self.timeout = self.declare_parameter('freshness_sec', 0.3).value
        self.ready = False
        self.bad = False
        self.fault_at = None
        self.fault_inputs = []
        self.good_since = None
        self.stationary_since = None
        self.last_actual = None
        self.base = None
        self.samples = {}
        self.input_health = {}
        self.fault_health = None
        self.last_tick = None
        self.last_tick_duration = None
        self.last_tick_gap = None
        self.fault_tick_gap = None
        self.shutdown_requested = False
        self.pub = self.create_publisher(ManualOperatorHeartbeat, 'gate/input/external_emergency_stop_heartbeat', 10)
        self.status = self.create_publisher(String, 'safety_status', 10)
        self.create_subscription(PointCloud2, 'obstacles/pointcloud', lambda m: self.receive('pointcloud', m.header.stamp), qos_profile_sensor_data)
        for stage in ['raw', 'extended', 'deskewed', 'compact']:
            self.create_subscription(PointCloud2, 'lidar/'+stage, lambda m, n=stage: self.receive(n, m.header.stamp), qos_profile_sensor_data)
        self.create_subscription(PredictedObjects, 'obstacles/predicted_objects', lambda m: self.receive('predicted_objects', m.header.stamp), 10)
        self.create_subscription(ManualOperatorHeartbeat, 'base_safety_heartbeat', self.heartbeat, 10)
        self.create_subscription(String, 'base_safety_status', self.base_status, 10)
        self.create_subscription(Odometry, 'sim/output/odometry', self.actual, 10)
        self.create_subscription(String, 'test_shutdown', lambda _: setattr(self, 'shutdown_requested', True), 10)
        self.create_service(Trigger, '~/rearm', self.rearm)
        self.create_timer(0.02, self.tick)

    def receive(self, name, source_stamp):
        mono=time.monotonic();source=stamp(source_stamp);ros_now=self.get_clock().now().nanoseconds*1e-9
        previous=self.samples.get(name)
        self.samples[name] = (source, mono, None if previous is None else mono-previous[1], ros_now-source)

    def base_status(self, msg):
        self.base = json.loads(msg.data)
        mono=time.monotonic();source=float(self.base['stamp']);ros_now=self.get_clock().now().nanoseconds*1e-9
        previous=self.samples.get('base_status')
        self.samples['base_status']=(source,mono,None if previous is None else mono-previous[1],ros_now-source)

    def heartbeat(self, msg):
        if msg.ready:
            self.receive('base_heartbeat', msg.stamp)

    def actual(self, msg):
        self.last_actual = time.monotonic()
        if abs(msg.twist.twist.linear.x) < 0.001:
            if self.stationary_since is None:
                self.stationary_since = self.last_actual
        else:
            self.stationary_since = None

    def stale(self):
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        mono = time.monotonic()
        names = ('raw', 'extended', 'deskewed', 'compact', 'pointcloud', 'predicted_objects', 'base_heartbeat', 'base_status')
        self.input_health = sample_health(self.samples, names, ros_now, mono)
        bad = [n for n in names
               if n not in self.samples or not 0 <= ros_now-self.samples[n][0] < self.timeout
               or mono-self.samples[n][1] >= self.timeout]
        if self.base is None or not self.base['armed'] or self.base['latched']:
            bad.append('base_authorization')
        return bad

    def rearm(self, _, response):
        now = time.monotonic()
        stopped = (self.stationary_since is not None and now-self.stationary_since >= 3
                   and self.last_actual is not None and now-self.last_actual < self.timeout)
        response.success = self.bad and not self.stale() and stopped
        if response.success:
            self.bad = False
            self.ready = False
            self.good_since = None
            self.fault_at = None
            self.fault_inputs = []
            response.message = 'Explicit rearm accepted; separately clear the pinned gate emergency.'
        else:
            response.message = 'Require a latched fault, all fresh inputs, base authorization and 3 s of live standstill.'
        return response

    def tick(self):
        tick_now = time.monotonic()
        tick_gap = tick_now-self.last_tick if self.last_tick is not None else None
        self.last_tick = tick_now
        self.last_tick_gap = tick_gap
        stale = self.stale()
        now = self.get_clock().now()
        if self.ready and stale and not self.bad:
            self.bad = True
            self.fault_at = now.nanoseconds * 1e-9
            self.fault_inputs = stale
            self.fault_health = self.input_health
            self.fault_tick_gap = tick_gap
        if stale:
            self.good_since = None
        elif not self.bad:
            if self.good_since is None:
                self.good_since = time.monotonic()
            if time.monotonic()-self.good_since >= 1:
                self.ready = True
                msg = ManualOperatorHeartbeat()
                msg.stamp = now.to_msg()
                msg.ready = True
                self.pub.publish(msg)
        msg = String()
        self.last_tick_duration = time.monotonic()-tick_now
        msg.data = json.dumps({'stamp': now.nanoseconds*1e-9, 'armed': self.ready and not self.bad,
                               'latched': self.bad, 'fault_stamp': self.fault_at,
                               'fault_inputs': self.fault_inputs, 'currently_stale': stale,
                               'fault_input_health': self.fault_health,
                               'fault_tick_gap_sec': self.fault_tick_gap,
                               'instrumentation': {'tick_schedule_gap_sec': tick_gap,
                                                   'tick_work_sec': self.last_tick_duration,
                                                   'inputs': self.input_health}})
        self.status.publish(msg)


def main():
    rclpy.init()
    node = Guard()
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
