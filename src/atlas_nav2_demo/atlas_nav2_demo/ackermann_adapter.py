"""Convert gated Nav2 curvature commands into bounded Autoware Control commands."""
import json, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from autoware_control_msgs.msg import Control
from std_msgs.msg import String


class Adapter(Node):
    def __init__(self):
        super().__init__('ackermann_adapter')
        self.wheelbase=float(self.declare_parameter('wheelbase_m',1.2).value)
        self.limit=float(self.declare_parameter('steering_limit_rad',.5).value)
        self.rate=float(self.declare_parameter('steering_rate_limit_radps',.6).value)
        self.speed_limit=float(self.declare_parameter('speed_limit_mps',.8).value)
        self.low_speed=float(self.declare_parameter('minimum_conversion_speed_mps',.05).value)
        if min(self.wheelbase,self.limit,self.rate,self.speed_limit,self.low_speed)<=0: raise ValueError('Ackermann limits must be positive')
        self.steering=0.; self.last=time.monotonic()
        self.pub=self.create_publisher(Control,'/atlas_ackermann/control_cmd',10)
        self.status=self.create_publisher(String,'/atlas_ackermann/command_status',10)
        self.create_subscription(Twist,'/atlas_ackermann/cmd_vel_safe',self.cb,10)
    def cb(self,msg):
        now=time.monotonic();dt=max(0.,now-self.last);self.last=now
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.get_logger().error('Rejected non-finite Nav2 command'); return
        velocity=max(-self.speed_limit,min(self.speed_limit,msg.linear.x))
        requested_curvature=0. if abs(velocity)<self.low_speed else msg.angular.z/velocity
        requested=0. if abs(velocity)<self.low_speed else math.atan(self.wheelbase*requested_curvature)
        # A controller command outside physical steering authority is not a
        # feasible Ackermann trajectory.  Stop instead of silently clipping
        # it and falsely reporting the path as executable.
        feasible=abs(requested)<=self.limit
        if not feasible:
            self.get_logger().error('Rejected infeasible curvature request')
            velocity=0.; requested=0.
        desired=requested
        delta=max(-self.rate*dt,min(self.rate*dt,desired-self.steering));self.steering+=delta
        out=Control();stamp=self.get_clock().now().to_msg();out.stamp=stamp;out.lateral.stamp=stamp;out.longitudinal.stamp=stamp
        out.lateral.steering_tire_angle=float(self.steering);out.lateral.steering_tire_rotation_rate=float(delta/dt) if dt>0 else 0.;out.lateral.is_defined_steering_tire_rotation_rate=True
        out.longitudinal.velocity=float(velocity);out.longitudinal.acceleration=0.;out.longitudinal.is_defined_acceleration=True
        self.pub.publish(out)
        self.status.publish(String(data=json.dumps({'feasible':feasible,'requested_curvature':requested_curvature,'applied_curvature':(math.tan(self.steering)/self.wheelbase),'requested_steering':requested,'applied_steering':self.steering},separators=(',',':'))))
def main():
 rclpy.init();n=Adapter()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.try_shutdown()
