#!/usr/bin/env python3
"""Simulation-only freshness latch: heartbeat/status only, no vehicle controls.
The pinned command gate supplies braking. Explicit rearm of both this monitor
and the pinned gate is required after a fault; recovered inputs alone do not arm.
"""
import json
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_adapi_v1_msgs.msg import ManualOperatorHeartbeat as Heartbeat
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Watchdog(Node):
    def __init__(self):
        super().__init__('feedback_watchdog')
        self.shutdown_requested=False
        self.create_subscription(String,'test_shutdown',lambda m:setattr(self,'shutdown_requested',True),10)
        self.stamps={}; self.receipts={}; self.bad=False; self.ready=False; self.good_since=None
        self.arm_stability=self.declare_parameter("arm_stability_sec",1.0).value
        self.timeout=self.declare_parameter('freshness_sec',.3).value
        self.fault_at=None; self.fault_inputs=[]; self.stationary_since=None
        self.pub=self.create_publisher(Heartbeat,'gate/input/external_emergency_stop_heartbeat',10)
        self.status=self.create_publisher(String,'safety_status',10)
        for name,typ,topic in [('odometry',Odometry,'feedback/odometry'),('steering',SteeringReport,'feedback/steering'),('acceleration',AccelWithCovarianceStamped,'feedback/acceleration'),('trajectory',Trajectory,'trajectory'),('command',Control,'control_cmd')]:
            self.create_subscription(typ,topic,lambda m,n=name:self.receive(n,m),10)
        self.create_subscription(Odometry,'sim/output/odometry',self.actual_state,10)
        self.create_service(Trigger,'~/rearm',self.rearm)
        self.create_timer(.02,self.tick)

    def receive(self,name,msg):
        stamp=msg.header.stamp if hasattr(msg,'header') else msg.stamp
        self.stamps[name]=stamp.sec+stamp.nanosec*1e-9
        self.receipts[name]=time.monotonic()

    def actual_state(self,msg):
        if abs(msg.twist.twist.linear.x)<.001:
            if self.stationary_since is None:self.stationary_since=time.monotonic()
        else:self.stationary_since=None

    def stale(self):
        now=self.get_clock().now().nanoseconds*1e-9
        mono=time.monotonic()
        return [n for n in ['odometry','steering','acceleration','trajectory','command']
                if n not in self.stamps or not 0<=now-self.stamps[n]<self.timeout
                or mono-self.receipts[n]>=self.timeout]

    def rearm(self,request,response):
        stopped=self.stationary_since is not None and time.monotonic()-self.stationary_since>=3.
        response.success=self.bad and not self.stale() and stopped
        if response.success:
            self.bad=False;self.ready=False;self.good_since=None;self.fault_at=None;self.fault_inputs=[]
            response.message='Monitor explicitly rearmed at standstill; separately clear pinned gate emergency to resume.'
        else:response.message='Rearm refused: require latched fault, fresh inputs and three seconds at standstill.'
        return response

    def tick(self):
        stale=self.stale()
        if not stale and not self.bad:
            if self.good_since is None:self.good_since=time.monotonic()
            if not self.ready and time.monotonic()-self.good_since<self.arm_stability:
                self.publish_status(stale);return
            self.ready=True
            hb=Heartbeat();hb.ready=True;hb.stamp=self.get_clock().now().to_msg();self.pub.publish(hb)
        elif self.ready and not self.bad:
            self.bad=True;self.fault_at=self.get_clock().now().nanoseconds*1e-9;self.fault_inputs=stale
            self.get_logger().warn('LATCHED stale required input: '+','.join(stale)+'; heartbeat withheld; pinned gate must brake')
        if stale:self.good_since=None
        self.publish_status(stale)

    def publish_status(self,stale):
        msg=String();msg.data=json.dumps({'stamp':self.get_clock().now().nanoseconds*1e-9,
            'armed':self.ready and not self.bad,'latched':self.bad,'fault_stamp':self.fault_at,
            'fault_inputs':self.fault_inputs,'currently_stale':stale,
            'stationary_since_monotonic':self.stationary_since})
        self.status.publish(msg)


def main():
    rclpy.init();node=Watchdog()
    try:
        while rclpy.ok() and not node.shutdown_requested:rclpy.spin_once(node,timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
