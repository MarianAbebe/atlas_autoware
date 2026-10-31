#!/usr/bin/env python3
"""Bounded standalone driver/recorder for profiling the full synthetic scene."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class Profile(Node):
    def __init__(self, duration):
        super().__init__('scene_profile_driver')
        self.duration=duration;self.started=time.monotonic();self.initialized=False;self.last_path_publish=-1.
        self.path_pub=self.create_publisher(PathMsg,'/atlas_loc/route_path',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom_pub=self.create_publisher(Odometry,'/atlas_loc/sim/output/odometry',10)
        self.create_subscription(String,'/atlas_loc/diagnostics/scene_timing',self.on_timing,10)
        self.create_subscription(PointCloud2,'/atlas_loc/lidar/raw',self.on_cloud,qos_profile_sensor_data)
        self.records=[];self.clouds=[];self.timer=self.create_timer(.02,self.tick)

    def on_timing(self,msg):
        record=json.loads(msg.data)
        self.records.append(dict(record,observer_receipt_monotonic_sec=time.monotonic()))
        if record.get('event')=='path_initialization':self.initialized=True

    def on_cloud(self,msg):
        now=self.get_clock().now().nanoseconds*1e-9
        self.clouds.append(dict(receipt_monotonic_sec=time.monotonic(),header_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            header_age_sec=now-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9),width=msg.width,
            point_step=msg.point_step,frame_id=msg.header.frame_id))

    def publish_path(self,now):
        msg=PathMsg();msg.header.frame_id='map';msg.header.stamp=now.to_msg()
        for i in range(81):
            p=PoseStamped();p.header=msg.header;p.pose.position.x=i*.25;p.pose.orientation.w=1.;msg.poses.append(p)
        self.path_pub.publish(msg)

    def tick(self):
        elapsed=time.monotonic()-self.started;now=self.get_clock().now()
        if not self.initialized and elapsed-self.last_path_publish>=.5:
            self.publish_path(now);self.last_path_publish=elapsed
        msg=Odometry();msg.header.frame_id='map';msg.child_frame_id='hidden_truth_base';msg.header.stamp=now.to_msg()
        speed=.25;msg.pose.pose.position.x=speed*elapsed;msg.pose.pose.position.y=.15*math.sin(.2*elapsed)
        yaw=math.atan2(.03*math.cos(.2*elapsed),speed)
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        msg.twist.twist.linear.x=speed;msg.twist.twist.angular.z=-.006*math.sin(.2*elapsed)
        self.odom_pub.publish(msg)
        if elapsed>=self.duration:self.timer.cancel()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=15.)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if not 5.<=args.duration<=60.:parser.error('duration must be between 5 and 60 seconds')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=Profile(args.duration)
    end=time.monotonic()+args.duration
    try:
        while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:
        (args.output/'scene_timing.json').write_text(json.dumps(node.records,indent=2))
        (args.output/'raw_receipts.json').write_text(json.dumps(node.clouds,indent=2))
        node.destroy_node();rclpy.try_shutdown()
    return 0 if node.clouds else 1


if __name__=='__main__':raise SystemExit(main())
