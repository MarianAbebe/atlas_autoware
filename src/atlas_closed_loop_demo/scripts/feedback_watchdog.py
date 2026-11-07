#!/usr/bin/env python3
"""Simulation-only freshness latch: heartbeat/status only, no vehicle controls.
The pinned command gate supplies braking. Explicit rearm of both this monitor
and the pinned gate is required after a fault; recovered inputs alone do not arm.
"""
import json
import threading
import time
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_adapi_v1_msgs.msg import ManualOperatorHeartbeat as Heartbeat
from std_msgs.msg import String
from std_srvs.srv import Trigger


def format_fault_detail(fault_health, tick_gap_sec):
    """Serialize the exact source/receipt freshness snapshot on a latch."""
    return json.dumps({
        'fault_input_health': fault_health or {},
        'fault_tick_gap_sec': tick_gap_sec,
    }, sort_keys=True, separators=(',', ':'))


class Watchdog(Node):
    def __init__(self):
        super().__init__('feedback_watchdog')
        # Keep the safety timer schedulable even when a large feedback message
        # is being deserialized or another input callback is queued. Inputs
        # remain mutually exclusive; the localization subclass runs NDT in a
        # third group so it cannot be held behind trajectory callbacks.
        self.state_lock=threading.RLock()
        self.input_callback_group=MutuallyExclusiveCallbackGroup()
        self.timer_callback_group=MutuallyExclusiveCallbackGroup()
        self.shutdown_requested=False
        self.create_subscription(String,'test_shutdown',lambda m:setattr(self,'shutdown_requested',True),10,
                                 callback_group=self.input_callback_group)
        self.stamps={}; self.receipts={}; self.bad=False; self.ready=False; self.good_since=None
        self.arm_stability=self.declare_parameter("arm_stability_sec",1.0).value
        self.timeout=self.declare_parameter('freshness_sec',.3).value
        self.fault_at=None; self.fault_inputs=[]; self.stationary_since=None
        self.input_health={};self.fault_health=None;self.last_tick=None;self.fault_tick_gap=None
        self.tick_count=0;self.last_tick_start_monotonic=None;self.last_tick_gap_sec=None
        self.max_tick_gap_sec=0.0;self.last_tick_duration_sec=None
        self.heartbeat_publish_count=0;self.last_heartbeat_publish_monotonic=None
        self.heartbeat_publish_duration_sec=None
        self.status_publish_count=0;self.last_status_publish_monotonic=None
        self.last_status_publish_call_start_monotonic=None
        self.status_publish_duration_sec=None
        self.status_publish_operation_duration_sec=None
        self.pub=self.create_publisher(Heartbeat,'gate/input/external_emergency_stop_heartbeat',10)
        self.status=self.create_publisher(String,'safety_status',10)
        for name,typ,topic in [('odometry',Odometry,'feedback/odometry'),('steering',SteeringReport,'feedback/steering'),('acceleration',AccelWithCovarianceStamped,'feedback/acceleration'),('trajectory',Trajectory,'trajectory'),('command',Control,'control_cmd')]:
            self.create_subscription(typ,topic,lambda m,n=name:self.receive(n,m),10,
                                     callback_group=self.input_callback_group)
        self.create_subscription(Odometry,'sim/output/odometry',self.actual_state,10,
                                 callback_group=self.input_callback_group)
        self.create_service(Trigger,'~/rearm',self.rearm,callback_group=self.input_callback_group)
        self.create_timer(.02,self.tick,callback_group=self.timer_callback_group)

    def receive(self,name,msg):
        stamp=msg.header.stamp if hasattr(msg,'header') else msg.stamp
        with self.state_lock:
            self.stamps[name]=stamp.sec+stamp.nanosec*1e-9
            self.receipts[name]=time.monotonic()

    def actual_state(self,msg):
        with self.state_lock:
            if abs(msg.twist.twist.linear.x)<.001:
                if self.stationary_since is None:self.stationary_since=time.monotonic()
            else:self.stationary_since=None

    def stale(self):
        now=self.get_clock().now().nanoseconds*1e-9
        mono=time.monotonic()
        with self.state_lock:
            names=['odometry','steering','acceleration','trajectory','command']
            self.input_health={n:dict(source_stamp=self.stamps.get(n),
                stamp_age_sec=now-self.stamps[n] if n in self.stamps else None,
                receipt_age_sec=mono-self.receipts[n] if n in self.receipts else None)
                for n in names}
            return [n for n in names
                    if n not in self.stamps or not 0<=now-self.stamps[n]<self.timeout
                    or mono-self.receipts[n]>=self.timeout]

    def rearm(self,request,response):
        with self.state_lock:
            stopped=self.stationary_since is not None and time.monotonic()-self.stationary_since>=3.
            response.success=self.bad and not self.stale() and stopped
            if response.success:
                self.bad=False;self.ready=False;self.good_since=None;self.fault_at=None;self.fault_inputs=[]
                response.message='Monitor explicitly rearmed at standstill; separately clear pinned gate emergency to resume.'
            else:response.message='Rearm refused: require latched fault, fresh inputs and three seconds at standstill.'
        return response

    def tick(self):
        tick_now=time.monotonic()
        tick_gap=tick_now-self.last_tick if self.last_tick is not None else None
        self.last_tick=tick_now
        self.tick_count+=1;self.last_tick_start_monotonic=tick_now
        self.last_tick_gap_sec=tick_gap
        if tick_gap is not None:self.max_tick_gap_sec=max(self.max_tick_gap_sec,tick_gap)
        with self.state_lock:
            stale=self.stale()
            if not stale and not self.bad:
                if self.good_since is None:self.good_since=time.monotonic()
                if self.ready or time.monotonic()-self.good_since>=self.arm_stability:
                    self.ready=True
                    hb=Heartbeat();hb.ready=True;hb.stamp=self.get_clock().now().to_msg()
                    # Publish while holding the short state lock so an input
                    # callback cannot invalidate the decision between check
                    # and authorization publication.
                    pub_start=time.monotonic();self.pub.publish(hb);pub_end=time.monotonic()
                    self.heartbeat_publish_count+=1;self.last_heartbeat_publish_monotonic=pub_start
                    self.heartbeat_publish_duration_sec=pub_end-pub_start
            elif self.ready and not self.bad:
                self.bad=True;self.fault_at=self.get_clock().now().nanoseconds*1e-9;self.fault_inputs=stale
                self.fault_health=dict(self.input_health);self.fault_tick_gap=tick_gap
                detail = format_fault_detail(self.fault_health, self.fault_tick_gap)
                self.get_logger().warn('LATCHED stale required input: '+','.join(stale)+
                    '; heartbeat withheld; pinned gate must brake; fault diagnostics: '+detail)
            if stale:self.good_since=None
        self.publish_status(stale)
        self.last_tick_duration_sec=time.monotonic()-tick_now

    def publish_status(self,stale):
        publish_start=time.monotonic()
        with self.state_lock:
            msg=String();msg.data=json.dumps({'stamp':self.get_clock().now().nanoseconds*1e-9,
                'armed':self.ready and not self.bad,'latched':self.bad,'fault_stamp':self.fault_at,
                'fault_inputs':self.fault_inputs,'currently_stale':stale,
                'stationary_since_monotonic':self.stationary_since,
                'fault_input_health':self.fault_health,'fault_tick_gap_sec':self.fault_tick_gap,
                'instrumentation':self.publication_diagnostics(publish_start)})
        # Do not hold the shared input-state lock across the RMW call. A slow
        # diagnostic publisher must not prevent feedback callbacks from
        # refreshing safety state.
        publish_call_start=time.monotonic()
        self.status.publish(msg)
        publish_end=time.monotonic()
        self.status_publish_count+=1;self.last_status_publish_monotonic=publish_start
        self.last_status_publish_call_start_monotonic=publish_call_start
        self.status_publish_duration_sec=publish_end-publish_call_start
        self.status_publish_operation_duration_sec=publish_end-publish_start

    def publication_diagnostics(self,status_publish_start=None):
        """Bounded publisher/timer diagnostics; times use this process monotonic clock."""
        return {'watchdog_tick_count':self.tick_count,
            'watchdog_tick_start_monotonic_sec':self.last_tick_start_monotonic,
            'watchdog_tick_gap_sec':self.last_tick_gap_sec,
            'watchdog_tick_max_gap_sec':self.max_tick_gap_sec,
            'watchdog_tick_previous_duration_sec':self.last_tick_duration_sec,
            'heartbeat_publish_count':self.heartbeat_publish_count,
            'heartbeat_last_publish_monotonic_sec':self.last_heartbeat_publish_monotonic,
            'heartbeat_publish_call_duration_sec':self.heartbeat_publish_duration_sec,
            'status_publish_count':self.status_publish_count+1,
            'status_publish_start_monotonic_sec':status_publish_start,
            'status_previous_publish_monotonic_sec':self.last_status_publish_monotonic,
            'status_previous_publish_call_start_monotonic_sec':self.last_status_publish_call_start_monotonic,
            'status_previous_publish_call_duration_sec':self.status_publish_duration_sec,
            'status_previous_publish_operation_duration_sec':self.status_publish_operation_duration_sec,
            'process_cpu_time_sec':time.process_time()}


def main():
    rclpy.init();node=Watchdog()
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.shutdown_requested:executor.spin_once(timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
