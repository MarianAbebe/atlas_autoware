"""Isolated ideal Ackermann bicycle simulator; never a physical actuator interface."""
import time, math
import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from .ackermann_kinematics import AckermannState,step
from .world import clearance, rectangles

class Simulator(Node):
 def __init__(self):
  super().__init__('ackermann_simulator');self.L=float(self.declare_parameter('wheelbase_m',1.2).value);self.limit=float(self.declare_parameter('steering_limit_rad',.5).value);self.rate=float(self.declare_parameter('steering_rate_limit_radps',.6).value);self.accel=float(self.declare_parameter('acceleration_limit_mps2',.8).value);self.decel=float(self.declare_parameter('deceleration_limit_mps2',1.2).value);self.vmax=float(self.declare_parameter('speed_limit_mps',.8).value);self.timeout=float(self.declare_parameter('command_timeout_sec',.25).value);self.scenario=self.declare_parameter('scenario','open').value;self.robot_radius=float(self.declare_parameter('robot_radius',.32).value);self.initial_x=float(self.declare_parameter('initial_x_m',-1.).value);self.initial_y=float(self.declare_parameter('initial_y_m',0.).value);rectangles(self.scenario);self.state=AckermannState(x=self.initial_x,y=self.initial_y);self.target=(0.,0.);self.last_cmd=None;self.last=time.monotonic();self.pos=0.
  self.odom=self.create_publisher(Odometry,'/atlas_ackermann/odom',20);self.enc=self.create_publisher(JointState,'/atlas_ackermann/encoder_states',20);self.tf=TransformBroadcaster(self);self.stf=StaticTransformBroadcaster(self);self.create_subscription(Control,'/atlas_ackermann/control_cmd',self.cb,10);self.static();self.create_timer(.02,self.tick)
 def static(self):
  now=self.get_clock().now().to_msg();a=TransformStamped();a.header.stamp=now;a.header.frame_id='map';a.child_frame_id='odom';a.transform.rotation.w=1.;b=TransformStamped();b.header.stamp=now;b.header.frame_id='base_link';b.child_frame_id='laser';b.transform.translation.x=.2;b.transform.translation.z=.25;b.transform.rotation.w=1.;self.stf.sendTransform([a,b])
 def cb(self,m):
  if not all(math.isfinite(v) for v in (m.longitudinal.velocity,m.lateral.steering_tire_angle)):self.get_logger().error('Rejected invalid Ackermann command');return
  self.target=(m.longitudinal.velocity,m.lateral.steering_tire_angle);self.last_cmd=time.monotonic()
 def tick(self):
  now=time.monotonic();dt=min(.05,max(0.,now-self.last));self.last=now;target=self.target if self.last_cmd and now-self.last_cmd<=self.timeout else (0.,0.);candidate,yaw_rate=step(self.state,*target,dt,self.L,self.limit,self.rate,self.accel,self.decel,self.vmax)
  if clearance(candidate.x,candidate.y,self.robot_radius,self.scenario)<0.: candidate=AckermannState(self.state.x,self.state.y,self.state.yaw,0.,candidate.steering);yaw_rate=0.
  self.state=candidate;self.pos+=self.state.speed*dt;stamp=self.get_clock().now().to_msg();qz=math.sin(self.state.yaw/2);qw=math.cos(self.state.yaw/2);o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_link';o.pose.pose.position.x=self.state.x;o.pose.pose.position.y=self.state.y;o.pose.pose.orientation.z=qz;o.pose.pose.orientation.w=qw;o.twist.twist.linear.x=self.state.speed;o.twist.twist.angular.z=yaw_rate;self.odom.publish(o);t=TransformStamped();t.header=o.header;t.child_frame_id='base_link';t.transform.translation.x=self.state.x;t.transform.translation.y=self.state.y;t.transform.rotation=o.pose.pose.orientation;self.tf.sendTransform(t);e=JointState();e.header.stamp=stamp;e.name=['rear_axle_encoder'];e.position=[self.pos];e.velocity=[self.state.speed];self.enc.publish(e)
def main():
 rclpy.init();n=Simulator()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.try_shutdown()
