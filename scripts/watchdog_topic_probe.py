#!/usr/bin/env python3
"""Passive latest-sample timestamp probe; publishes nothing; use the test ROS domain."""
import argparse
import csv
from pathlib import Path
import time
import rclpy
from rclpy.qos import QoSProfile
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory

class TimingExecutor(SingleThreadedExecutor):
    """Humble diagnostic adapter: retain the metadata its executor discards.

    Times describe this observer's reader, not the watchdog reader. Middleware
    source time is publication time, distinct from the application header stamp.
    Zero/unavailable metadata is left blank rather than treated as a real time.
    """
    def _take_subscription(self, sub):
        with sub.handle:
            result = sub.handle.take_message(sub.msg_type, sub.raw)
        if result is not None:
            self.message_info = result[1]
            return result[0]
        return None

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=140)
    a=p.parse_args()
    if not 0<a.duration<=1200:p.error('duration must be >0 and <=1200 seconds')
    rclpy.init();node=rclpy.create_node('passive_watchdog_topic_probe')
    executor=TimingExecutor();executor.add_node(node)
    with a.output.open('x') as stream:
        writer=csv.writer(stream);writer.writerow(['topic','source_stamp','ros_receipt','monotonic_receipt',
            'middleware_source_ns','middleware_received_ns','callback_wall_ns'])
        def observe(topic,msg):
            stamp=msg.header.stamp if hasattr(msg,'header') else msg.stamp
            callback_wall_ns=time.time_ns()
            info=executor.message_info
            writer.writerow([topic,stamp.sec+stamp.nanosec*1e-9,node.get_clock().now().nanoseconds*1e-9,time.monotonic(),
                info.get('source_timestamp') or '',info.get('received_timestamp') or '',callback_wall_ns])
        subscriptions=[]
        for topic,typ in [('sim/output/odometry',Odometry),('sim/output/steering',SteeringReport),
            ('sim/output/acceleration',AccelWithCovarianceStamped),('feedback/odometry',Odometry),
            ('feedback/steering',SteeringReport),('feedback/acceleration',AccelWithCovarianceStamped),
            ('controller_raw/control_cmd',Control),('control_cmd',Control),('trajectory',Trajectory)]:
            subscriptions.append(node.create_subscription(typ,'/atlas_closed_loop/'+topic,
                lambda msg,t=topic:observe(t,msg),QoSProfile(depth=1)))
        until=time.monotonic()+a.duration
        try:
            while rclpy.ok() and time.monotonic()<until:
                executor.spin_once(timeout_sec=.02)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    executor.shutdown();node.destroy_node();rclpy.try_shutdown()


if __name__ == "__main__":
    main()
