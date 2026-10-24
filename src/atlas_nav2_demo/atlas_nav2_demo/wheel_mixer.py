"""Convert gated Twist commands to four skid-steer wheel targets."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .kinematics import mix_twist


WHEEL_NAMES = ['front_left_wheel', 'rear_left_wheel',
               'front_right_wheel', 'rear_right_wheel']


class WheelMixer(Node):
    def __init__(self):
        super().__init__('wheel_mixer')
        self.radius = self.declare_parameter('wheel_radius', 0.15).value
        self.track = self.declare_parameter('track_width', 0.60).value
        mix_twist(0.0, 0.0, self.track, self.radius)
        self.publisher = self.create_publisher(
            JointState, '/atlas_nav2/wheel_speed_targets', 10)
        self.create_subscription(Twist, '/atlas_nav2/cmd_vel_safe', self.on_twist, 10)

    def on_twist(self, msg):
        speeds = mix_twist(msg.linear.x, msg.angular.z, self.track, self.radius)
        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = WHEEL_NAMES
        output.velocity = [speeds.front_left, speeds.rear_left,
                           speeds.front_right, speeds.rear_right]
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = WheelMixer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
