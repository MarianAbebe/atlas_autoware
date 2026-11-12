"""Convert gated Nav2 curvature commands into bounded Autoware Control commands."""
import json, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from autoware_control_msgs.msg import Control
from std_msgs.msg import String
from .ackermann_adapter_logic import convert_command


class Adapter(Node):
    def __init__(self):
        super().__init__('ackermann_adapter')
        self.wheelbase=float(self.declare_parameter('wheelbase_m',1.2).value)
        self.limit=float(self.declare_parameter('steering_limit_rad',.5).value)
        self.rate=float(self.declare_parameter('steering_rate_limit_radps',.6).value)
        self.speed_limit=float(self.declare_parameter('speed_limit_mps',.8).value)
        self.low_speed=float(self.declare_parameter('minimum_conversion_speed_mps',.05).value)
        if min(self.wheelbase,self.limit,self.rate,self.speed_limit,self.low_speed)<=0: raise ValueError('Ackermann limits must be positive')
        self.steering=0.; self.last=time.monotonic(); self.last_reject_log=0.0
        self.pub=self.create_publisher(Control,'/atlas_ackermann/control_cmd',10)
        self.status=self.create_publisher(String,'/atlas_ackermann/command_status',10)
        self.create_subscription(Twist,'/atlas_ackermann/cmd_vel_safe',self.cb,10)
    def cb(self,msg):
        now_ns=time.monotonic_ns();now=now_ns*1e-9;dt=max(0.,now-self.last);self.last=now
        velocity,requested_curvature,requested,feasible=convert_command(
            msg.linear.x,msg.angular.z,self.wheelbase,self.limit,
            self.speed_limit,self.low_speed)
        # A controller command outside physical steering authority is not a
        # feasible Ackermann trajectory.  Stop instead of silently clipping
        # it and falsely reporting the path as executable.
        if not feasible:
            if now-self.last_reject_log >= 1.0:
                self.get_logger().warning(
                    'Rejecting infeasible Ackermann request: '
                    f'curvature={requested_curvature:.3f} 1/m, '
                    f'steering={requested:.3f} rad')
                self.last_reject_log=now
        desired=requested if feasible else 0.0
        delta=max(-self.rate*dt,min(self.rate*dt,desired-self.steering));self.steering+=delta
        out=Control();stamp=self.get_clock().now().to_msg();out.stamp=stamp;out.lateral.stamp=stamp;out.longitudinal.stamp=stamp
        out.lateral.steering_tire_angle=float(self.steering);out.lateral.steering_tire_rotation_rate=float(delta/dt) if dt>0 else 0.;out.lateral.is_defined_steering_tire_rotation_rate=True
        out.longitudinal.velocity=float(velocity);out.longitudinal.acceleration=0.;out.longitudinal.is_defined_acceleration=True
        self.pub.publish(out)
        publish_mono_ns=time.monotonic_ns()
        requested_speed = msg.linear.x if math.isfinite(msg.linear.x) else None
        self.status.publish(String(data=json.dumps({'feasible':feasible,'requested_curvature':requested_curvature,'applied_curvature':(math.tan(self.steering)/self.wheelbase),'requested_steering':requested,'applied_steering':self.steering,'requested_linear_velocity':requested_speed,'applied_linear_velocity':velocity,'adapter_input_callback_monotonic_ns':now_ns,'control_publish_monotonic_ns':publish_mono_ns},separators=(',',':'))))
def main():
 rclpy.init();n=Adapter()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.try_shutdown()
