"""Simulation-only, default-disabled velocity gate for the Nav2 demo."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .gate_logic import gated_velocity


class SafetyGate(Node):
    def __init__(self):
        super().__init__('safety_gate')
        self.timeout = self.declare_parameter('command_timeout_sec', 0.25).value
        self.max_linear = self.declare_parameter('max_linear_velocity', 0.55).value
        self.max_angular = self.declare_parameter('max_angular_velocity', 1.2).value
        if self.timeout <= 0.0 or self.max_linear <= 0.0 or self.max_angular <= 0.0:
            raise ValueError('gate timeout and velocity limits must be positive')
        self.enabled = False
        self.emergency_stop = False
        self.command = Twist()
        self.last_command = None
        self.output = self.create_publisher(Twist, '/atlas_nav2/cmd_vel_safe', 10)
        self.status = self.create_publisher(String, '/atlas_nav2/safety_status', 10)
        self.create_subscription(Twist, '/atlas_nav2/cmd_vel_nav', self.on_command, 10)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_service(SetBool, '~/emergency_stop', self.on_emergency_stop)
        self.timer = self.create_timer(0.02, self.tick)

    def on_command(self, msg):
        values = (msg.linear.x, msg.angular.z)
        if all(math.isfinite(value) for value in values):
            self.command = msg
            self.last_command = time.monotonic()
        else:
            self.emergency_stop = True
            self.get_logger().error('Non-finite velocity command latched emergency stop')

    def on_enable(self, request, response):
        if request.data and self.emergency_stop:
            response.success = False
            response.message = 'emergency stop is active'
            return response
        self.enabled = request.data
        response.success = True
        response.message = 'simulation gate enabled' if self.enabled else 'simulation gate disabled'
        if not self.enabled:
            self.publish_zero()
        return response

    def on_emergency_stop(self, request, response):
        self.emergency_stop = request.data
        if self.emergency_stop:
            self.enabled = False
            self.publish_zero()
        response.success = True
        response.message = 'emergency stop active' if request.data else 'emergency stop cleared while gate remains disabled'
        return response

    def publish_zero(self):
        self.output.publish(Twist())

    def tick(self):
        now = time.monotonic()
        age = math.inf if self.last_command is None else now - self.last_command
        fresh = age <= self.timeout
        output = Twist()
        if math.isfinite(age):
            output.linear.x, output.angular.z = gated_velocity(
                self.enabled, self.emergency_stop, age, self.timeout,
                self.command.linear.x, self.command.angular.z,
                self.max_linear, self.max_angular)
        self.output.publish(output)
        self.status.publish(String(data=json.dumps({
            'enabled': self.enabled,
            'emergency_stop': self.emergency_stop,
            'command_fresh': fresh,
            'output_linear': output.linear.x,
            'output_angular': output.angular.z,
        }, separators=(',', ':'))))

    def destroy_node(self):
        # A SIGINT can invalidate rclpy before launch calls this method.  The
        # simulator independently times out wheel targets, so never turn that
        # shutdown race into a nonzero command or an exception.
        if rclpy.ok():
            for _ in range(3):
                self.publish_zero()
        return super().destroy_node()


def main():
    rclpy.init()
    node = SafetyGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
