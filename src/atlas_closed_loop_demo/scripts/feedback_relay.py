#!/usr/bin/env python3
"""Unmodified simulator feedback relay with explicit fault-injection modes.
Controller commands are forwarded unchanged, except explicit drop/replay fault injection. No control commands are generated here. DRIVE, initialization and operation
mode are test scaffolding. drop/freeze affect only the selected required input.
"""
import copy
import time
import math
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import AccelWithCovarianceStamped, PoseWithCovarianceStamped, PoseStamped
from autoware_planning_msgs.msg import LaneletRoute
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand, SteeringReport, Engage
from autoware_adapi_v1_msgs.msg import OperationModeState
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
from command_recording import CommandJournal
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class Relay(Node):
    def __init__(self):
        super().__init__('feedback_relay')
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.shutdown_requested=False
        self.create_subscription(String,'test_shutdown',lambda m:setattr(self,'shutdown_requested',True),10)
        self.planning_client=self.create_client(SetParameters,'path_generator/set_parameters')
        self.rear_support_request=None;self.rear_support_applied=False
        self.rear_support_distance=self.declare_parameter('rear_support_distance_m',10.).value
        self.rear_support_length=self.declare_parameter('terminal_backward_path_m',5.).value
        self.release_delay=self.declare_parameter("release_delay_sec",0.).value;self.created=time.monotonic();self.released=False
        self.require_safety=self.declare_parameter('require_safety',False).value;self.safety_armed=False;self.safety_stamp=0.
        self.create_subscription(String,'safety_status',self.on_safety,10)
        self.ep=self.create_publisher(Engage,"sim/input/engage",10)
        self.gep=self.create_publisher(Engage,"gate/input/engage",10)
        self.odom=None; self.route=None; self.inject='live'; self.frozen=None; self.initialized=False
        self.fault_input='odometry'; self.originals={}; self.fault_snapshot=None
        self.command_journal = CommandJournal() if self.declare_parameter('record_commands',False).value else None
        if self.command_journal is not None:
            self.create_service(Trigger, 'command_recording/snapshot', self.command_journal.snapshot)
        self.cp=self.create_publisher(Control,'control_cmd',10)
        self.create_subscription(Control,'controller_raw/control_cmd',lambda m:self.forward('command',m,self.cp),10)
        self.op=self.create_publisher(Odometry,'feedback/odometry',10)
        self.ap=self.create_publisher(AccelWithCovarianceStamped,'feedback/acceleration',10)
        self.sp=self.create_publisher(SteeringReport,'feedback/steering',10)
        self.mp=self.create_publisher(OperationModeState,'operation_mode',latched)
        self.gp=self.create_publisher(GearCommand,'sim/input/gear_command',10)
        self.agp=self.create_publisher(GearCommand,'auto_gear',10)
        self.ip=self.create_publisher(PoseWithCovarianceStamped,'sim/input/initialpose',10)
        self.pathpub=self.create_publisher(Path,'driven_path',10)
        self.markpub=self.create_publisher(MarkerArray,'vehicle_markers',10)
        self.last_view=0.
        self.path=Path();self.path.header.frame_id='map'
        self.create_subscription(Odometry,'sim/output/odometry',self.on_odom,10)
        self.create_subscription(AccelWithCovarianceStamped,'sim/output/acceleration',self.on_accel,10)
        self.create_subscription(SteeringReport,'sim/output/steering',lambda m:self.forward('steering',m,self.sp),10)
        self.create_subscription(LaneletRoute,'route',self.on_route,latched)
        self.create_subscription(String,'inject_feedback',self.on_inject,10)
        self.create_timer(.02,self.tick)
    def on_safety(self,msg):
        state=json.loads(msg.data);self.safety_armed=state['armed'];self.safety_stamp=state['stamp']

    def on_accel(self,m):
        # Pinned simulator hardcodes /base_link. Rename the identical body axes
        # to our isolated frame; retain every value and the original timestamp.
        m.header.frame_id="atlas_closed_loop_base_link";self.ap.publish(m)
    def on_route(self,m): self.route=m
    def on_inject(self,m):
        parts=m.data.split(':'); self.inject=parts[0]; self.fault_input=parts[1] if len(parts)>1 else 'odometry'
        self.fault_snapshot=copy.deepcopy(self.originals.get(self.fault_input))
        self.get_logger().warn('Fault injection: '+m.data+' '+self.fault_input+' (original stamps retained)')
    def forward(self,name,msg,publisher):
        if name == 'command' and getattr(self, 'command_journal', None) is not None:
            self.command_journal.record(msg)
        self.originals[name]=msg
        if self.inject=='live' or name!=self.fault_input:
            publisher.publish(msg)
        elif self.inject=='freeze' and self.fault_snapshot is not None:
            publisher.publish(self.fault_snapshot)

    def on_odom(self,m):
        self.odom=m
        if self.route and self.rear_support_request is None and math.hypot(m.pose.pose.position.x-self.route.goal_pose.position.x,m.pose.pose.position.y-self.route.goal_pose.position.y)<self.rear_support_distance and self.planning_client.service_is_ready():
            req=SetParameters.Request()
            req.parameters=[Parameter(name='path_length.backward',value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=self.rear_support_length))]
            self.rear_support_request=self.planning_client.call_async(req)
            self.rear_support_request.add_done_callback(self.on_rear_support)
        self.forward('odometry',m,self.op)
        pose=PoseStamped();pose.header=m.header;pose.pose=m.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses)>12000:self.path.poses=self.path.poses[-12000:]
    def on_rear_support(self,future):
        response=future.result()
        self.rear_support_applied=bool(response.results and all(r.successful for r in response.results))
        if self.rear_support_applied:
            self.get_logger().info('Terminal spline support: backward path set to %.1f m below %.1f m remaining; initial validated plan unchanged' % (self.rear_support_length,self.rear_support_distance))
        else:self.get_logger().error('Failed to extend terminal spline support')

    def tick(self):
        now=self.get_clock().now().to_msg()
        if self.route and not self.initialized:
            # Send once after simulator is discovered, never repeatedly reset a live vehicle.
            if self.ip.get_subscription_count():
                p=PoseWithCovarianceStamped();p.header.frame_id='map';p.header.stamp=now
                p.pose.pose=self.route.start_pose;self.ip.publish(p);self.initialized=True
        mode=OperationModeState();mode.stamp=now;mode.mode=mode.AUTONOMOUS
        mode.is_autoware_control_enabled=True;self.mp.publish(mode)
        if self.initialized and not self.released and time.monotonic()-self.created>=self.release_delay and (not self.require_safety or (self.safety_armed and 0<=self.get_clock().now().nanoseconds*1e-9-self.safety_stamp<.3)):
            e=Engage();e.stamp=now;e.engage=True;self.ep.publish(e);self.released=True
        e=Engage();e.stamp=now;e.engage=self.initialized;self.gep.publish(e)
        gear=GearCommand();gear.stamp=now;gear.command=gear.DRIVE;self.gp.publish(gear);self.agp.publish(gear)
        if self.odom and time.monotonic()-self.last_view>=.2:
            self.last_view=time.monotonic()
            self.path.header.stamp=now;self.pathpub.publish(self.path)
            m=Marker();m.header=self.odom.header;m.ns='simulated_atlas';m.id=0;m.type=m.CUBE;m.action=m.ADD
            m.pose=self.odom.pose.pose;m.scale.x=1.8;m.scale.y=1.;m.scale.z=.5
            m.color.r=1.;m.color.g=.4;m.color.a=1.
            self.markpub.publish(MarkerArray(markers=[m]))

def main():
    rclpy.init();n=Relay()
    try:
        while rclpy.ok() and not n.shutdown_requested:rclpy.spin_once(n,timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
