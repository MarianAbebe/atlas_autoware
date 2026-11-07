"""Animate a visualization-only rover from the authoritative simulator state."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from autoware_control_msgs.msg import Control

class ShowcaseJoints(Node):
    def __init__(self):
        super().__init__('ackermann_showcase_joints'); self.steer=0.; self.wheel=0.; self.speed=0.; self.last=self.get_clock().now()
        self.pub=self.create_publisher(JointState,'/atlas_ackermann/showcase_joint_states',10)
        self.create_subscription(Control,'/atlas_ackermann/control_cmd',self.control,10)
        self.create_timer(.04,self.tick)
    def control(self,msg): self.steer=msg.lateral.steering_tire_angle; self.speed=msg.longitudinal.velocity
    def tick(self):
        now=self.get_clock().now(); dt=max(0.,(now-self.last).nanoseconds/1e9); self.last=now; self.wheel+=self.speed*dt/.22
        m=JointState();m.header.stamp=now.to_msg();m.name=['front_left_steering_joint','front_right_steering_joint','front_left_wheel_joint','front_right_wheel_joint','rear_left_wheel_joint','rear_right_wheel_joint'];m.position=[self.steer,self.steer,self.wheel,self.wheel,self.wheel,self.wheel];self.pub.publish(m)
def main():
 rclpy.init();n=ShowcaseJoints()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.try_shutdown()
