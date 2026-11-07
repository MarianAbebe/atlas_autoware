#!/usr/bin/env python3
"""Exercise scene callbacks with deterministic synthetic path/odometry input.

This is a scene-process stress test, not a vehicle simulator or autonomy trial.
It must not be used as evidence of simulator, localization, or controller health.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def append_jsonl(path, row):
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n')
        stream.flush()


class Probe(Node):
    def __init__(self, duration_sec, output):
        super().__init__('atlas_scene_sensor_stress_probe')
        self.duration_sec=duration_sec;self.output=output;self.started=time.monotonic()
        transient=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_pub=self.create_publisher(RosPath,'/atlas_loc/route_path',transient)
        self.odom_pub=self.create_publisher(Odometry,'/atlas_loc/sim/output/odometry',1)
        self.create_subscription(PointCloud2,'/atlas_loc/lidar/raw',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(String,'/atlas_loc/diagnostics/scene_timing',self.on_timing,20)
        self.create_subscription(Odometry,'/atlas_loc/sim/output/odometry',self.on_odom_echo,50)
        self.scan_count=0;self.timing_count=0;self.last_scan_receipt=None
        self.last_timing_receipt=None;self.scan_gaps=[];self.timing_gaps=[]
        self.odom_publish_count=0;self.last_odom_publish=None;self.odom_publish_gaps=[]
        self.odom_echo_count=0;self.last_odom_echo=None;self.odom_echo_gaps=[]
        self.first_scan_stamp_ns=None;self.last_scan_stamp_ns=None
        self.scene_pids=set();self.scene_tick_indexes=[];self.skip_counts={}
        self.route_publications=0
        self.create_timer(.02,self.publish_odom)
        self.create_timer(.5,self.publish_route)

    def publish_route(self):
        elapsed=time.monotonic()-self.started
        if elapsed>4.0:return
        msg=RosPath();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        for index in range(41):
            pose=PoseStamped();pose.header=msg.header
            pose.pose.position.x=index*.5;pose.pose.orientation.w=1.0
            msg.poses.append(pose)
        self.route_pub.publish(msg);self.route_publications+=1

    def publish_odom(self):
        publish_mono=time.monotonic()
        if self.last_odom_publish is not None:
            self.odom_publish_gaps.append(publish_mono-self.last_odom_publish)
        self.last_odom_publish=publish_mono;self.odom_publish_count+=1
        now=self.get_clock().now();elapsed=time.monotonic()-self.started
        msg=Odometry();msg.header.stamp=now.to_msg();msg.header.frame_id='map';msg.child_frame_id='base_link'
        # Deterministic low-speed motion exercises moving-scan coordinate transforms.
        msg.pose.pose.position.x=.5*elapsed
        msg.pose.pose.position.y=.5*math.sin(elapsed/8.0)
        yaw=math.atan2(.0625*math.cos(elapsed/8.0),.5)
        msg.pose.pose.orientation.z=math.sin(yaw/2.0);msg.pose.pose.orientation.w=math.cos(yaw/2.0)
        msg.twist.twist.linear.x=.5;msg.twist.twist.linear.y=.0625*math.cos(elapsed/8.0)
        msg.twist.twist.angular.z=-.0078125*math.sin(elapsed/8.0)
        self.odom_pub.publish(msg)

    def on_odom_echo(self,msg):
        receipt_mono=time.monotonic()
        if self.last_odom_echo is not None:
            self.odom_echo_gaps.append(receipt_mono-self.last_odom_echo)
        self.last_odom_echo=receipt_mono;self.odom_echo_count+=1

    def on_scan(self,msg):
        now=time.monotonic()
        if self.last_scan_receipt is not None:self.scan_gaps.append(now-self.last_scan_receipt)
        self.last_scan_receipt=now;self.scan_count+=1
        stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if self.first_scan_stamp_ns is None:self.first_scan_stamp_ns=stamp_ns
        self.last_scan_stamp_ns=stamp_ns

    def on_timing(self,msg):
        now=time.monotonic()
        if self.last_timing_receipt is not None:self.timing_gaps.append(now-self.last_timing_receipt)
        self.last_timing_receipt=now;self.timing_count+=1
        try:record=json.loads(msg.data)
        except (TypeError,ValueError):return
        if record.get('process_pid') is not None:self.scene_pids.add(int(record['process_pid']))
        if record.get('scene_tick_index') is not None:self.scene_tick_indexes.append(int(record['scene_tick_index']))
        reason=record.get('skip_reason')
        if reason:self.skip_counts[reason]=self.skip_counts.get(reason,0)+1
        record['collector_receipt_monotonic_sec']=now
        append_jsonl(self.output/'scene_timing.jsonl',record)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=180.0)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 30.0<=args.duration<=600.0:parser.error('--duration must be between 30 and 600 seconds')
    if args.output.exists():parser.error(f'output already exists: {args.output}')
    args.output.mkdir(parents=True)
    scene_path=Path(get_package_prefix('atlas_full_stack_demo'))/'lib/atlas_full_stack_demo/scene.py'
    if not scene_path.is_file():parser.error(f'installed scene executable not found: {scene_path}')
    rclpy.init();probe=Probe(args.duration,args.output)
    log=(args.output/'scene_runtime.log').open('w',encoding='utf-8')
    process=None;exit_reason='duration_complete';return_code=None
    start_mono=time.monotonic();start_ros=probe.get_clock().now().nanoseconds
    append_jsonl(args.output/'process_lifecycle.jsonl',dict(event='probe_started',pid=os.getpid(),
        scene_executable=str(scene_path),scene_pid=None,monotonic_sec=start_mono,ros_time_ns=start_ros))
    try:
        process=subprocess.Popen([sys.executable,str(scene_path),'--ros-args','-p','obstacle:=false',
            '-p','request_engagement:=false','-p','route_length_m:=16.0'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        append_jsonl(args.output/'process_lifecycle.jsonl',dict(event='scene_process_started',
            pid=process.pid,process_group_id=process.pid,monotonic_sec=time.monotonic(),ros_time_ns=start_ros))
        deadline=start_mono+args.duration
        while rclpy.ok() and time.monotonic()<deadline and process.poll() is None:
            rclpy.spin_once(probe,timeout_sec=.01)
        if process.poll() is not None:exit_reason='scene_process_exited_early'
    except KeyboardInterrupt:
        exit_reason='probe_keyboard_interrupt'
    except BaseException as exc:
        exit_reason='probe_exception'
        append_jsonl(args.output/'process_lifecycle.jsonl',dict(event=exit_reason,
            exception_type=type(exc).__name__,exception=str(exc),monotonic_sec=time.monotonic()))
        raise
    finally:
        if process is not None and process.poll() is None:
            append_jsonl(args.output/'process_lifecycle.jsonl',dict(event='scene_signal_sent',signal='SIGINT',
                pid=process.pid,process_group_id=process.pid,monotonic_sec=time.monotonic()))
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                append_jsonl(args.output/'process_lifecycle.jsonl',dict(event='scene_signal_sent',signal='SIGTERM',
                    pid=process.pid,process_group_id=process.pid,monotonic_sec=time.monotonic()))
                try:os.killpg(process.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:exit_reason='scene_process_cleanup_timeout'
        if process is not None:return_code=process.returncode
        log.close()
        append_jsonl(args.output/'process_lifecycle.jsonl',dict(event='scene_process_exit',
            pid=None if process is None else process.pid,return_code=return_code,reason=exit_reason,
            monotonic_sec=time.monotonic(),ros_time_ns=probe.get_clock().now().nanoseconds))
        rclpy.try_shutdown()
        probe.destroy_node()

    duration=time.monotonic()-start_mono
    max_scan_gap=max(probe.scan_gaps) if probe.scan_gaps else None
    max_timing_gap=max(probe.timing_gaps) if probe.timing_gaps else None
    max_odom_publish_gap=max(probe.odom_publish_gaps) if probe.odom_publish_gaps else None
    max_odom_echo_gap=max(probe.odom_echo_gaps) if probe.odom_echo_gaps else None
    result=dict(status='PASS' if exit_reason=='duration_complete' and process is not None
        and return_code in (0,-signal.SIGINT) and probe.scan_count>=args.duration*8.0
        and max_scan_gap is not None and max_scan_gap<.3 else 'FAIL',
        semantics='scene process with deterministic synthetic 50 Hz odometry stimulus; not vehicle-simulator/autonomy validation',
        requested_duration_sec=args.duration,observed_duration_sec=duration,
        installed_scene_path=str(scene_path),scene_pid=process.pid if process else None,
        scene_return_code=return_code,scene_timing_records=probe.timing_count,raw_scan_count=probe.scan_count,
        raw_scan_rate_hz=probe.scan_count/duration if duration else None,
        max_raw_scan_receipt_gap_sec=max_scan_gap,diagnostic_rate_hz=probe.timing_count/duration if duration else None,
        odometry_publish_count=probe.odom_publish_count,
        max_odometry_publish_callback_gap_sec=max_odom_publish_gap,
        odometry_echo_count=probe.odom_echo_count,
        max_odometry_echo_callback_gap_sec=max_odom_echo_gap,
        max_scene_odometry_callback_gap_sec=max((r.get('odometry_callback_last_gap_sec') or 0.0
            for r in (json.loads(x) for x in (args.output/'scene_timing.jsonl').read_text().splitlines())
            if r.get('odometry_callback_last_gap_sec') is not None),default=None),
        max_diagnostic_receipt_gap_sec=max_timing_gap,first_scan_stamp_ns=probe.first_scan_stamp_ns,
        last_scan_stamp_ns=probe.last_scan_stamp_ns,scene_pids=sorted(probe.scene_pids),
        last_scene_tick_index=max(probe.scene_tick_indexes) if probe.scene_tick_indexes else None,
        skip_counts=probe.skip_counts,route_publications=probe.route_publications)
    (args.output/'validation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
