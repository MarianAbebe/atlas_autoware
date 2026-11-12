"""Simulation-only, default-disabled velocity gate for the Nav2 demo."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
        self.last_command_mono_ns = None
        self.last_localization = None
        self.last_localization_mono_ns = None
        self.last_localization_source_sec = None
        self.output = self.create_publisher(Twist, '/atlas_nav2/cmd_vel_safe', 10)
        self.status = self.create_publisher(String, '/atlas_nav2/safety_status', 10)
        self.create_subscription(Twist, '/atlas_nav2/cmd_vel_nav', self.on_command, 10)
        # This isolated Nav2 fixture treats /odom as its authoritative
        # estimator output.  The gate checks delivery freshness only; it never
        # manufactures timestamps or falls back to simulator state.
        self.create_subscription(Odometry, '/atlas_nav2/odom', self.on_localization, 20)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_service(SetBool, '~/emergency_stop', self.on_emergency_stop)
        self.timer = self.create_timer(0.02, self.tick)

    def on_command(self, msg):
        values = (msg.linear.x, msg.angular.z)
        if all(math.isfinite(value) for value in values):
            self.command = msg
            self.last_command_mono_ns = time.monotonic_ns()
            self.last_command = self.last_command_mono_ns * 1.0e-9
        else:
            self.emergency_stop = True
            self.enabled = False
            self.publish_zero()
            self.get_logger().error('Non-finite velocity command latched emergency stop')

    def on_localization(self, msg):
        stamp = msg.header.stamp
        if stamp.sec < 0 or stamp.nanosec < 0 or stamp.nanosec >= 1_000_000_000:
            self.get_logger().error('Rejected localization with invalid source timestamp')
            return
        source_sec = stamp.sec + stamp.nanosec * 1e-9
        source_age = self.get_clock().now().nanoseconds * 1e-9 - source_sec
        if not 0.0 <= source_age <= self.timeout:
            self.get_logger().error('Rejected stale or future localization source timestamp')
            return
        self.last_localization_mono_ns = time.monotonic_ns()
        self.last_localization = self.last_localization_mono_ns * 1.0e-9
        self.last_localization_source_sec = source_sec

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
        tick_mono_ns = time.monotonic_ns()
        now = tick_mono_ns * 1.0e-9
        age = math.inf if self.last_command is None else now - self.last_command
        localization_age = (math.inf if self.last_localization is None
                            else now - self.last_localization)
        localization_source_age = (
            math.inf if self.last_localization_source_sec is None else
            self.get_clock().now().nanoseconds * 1e-9 - self.last_localization_source_sec)
        fresh = age <= self.timeout
        localization_fresh = (localization_age <= self.timeout
                              and 0.0 <= localization_source_age <= self.timeout)
        output = Twist()
        if math.isfinite(age) and localization_fresh:
            output.linear.x, output.angular.z = gated_velocity(
                self.enabled, self.emergency_stop, age, localization_age, self.timeout,
                self.command.linear.x, self.command.angular.z,
                self.max_linear, self.max_angular)
        self.output.publish(output)
        output_publish_mono_ns = time.monotonic_ns()
        self.status.publish(String(data=json.dumps({
            'enabled': self.enabled,
            'emergency_stop': self.emergency_stop,
            'command_fresh': fresh,
            'localization_fresh': localization_fresh,
            # Keep diagnostic JSON standards-compliant when localization has
            # not yet been observed; JSON has no portable infinity literal.
            'localization_receipt_age_sec': (
                localization_age if math.isfinite(localization_age) else None),
            'localization_source_age_sec': (
                localization_source_age if math.isfinite(localization_source_age) else None),
            'output_linear': output.linear.x,
            'output_angular': output.angular.z,
            # Embedded monotonic stamps separate callback/timer delays from
            # downstream ROS delivery and recorder receipt delays.
            'gate_tick_monotonic_ns': tick_mono_ns,
            'safe_output_publish_monotonic_ns': output_publish_mono_ns,
            'last_command_receipt_monotonic_ns': self.last_command_mono_ns,
            'last_localization_receipt_monotonic_ns':
                self.last_localization_mono_ns,
            'command_age_monotonic_sec': age if math.isfinite(age) else None,
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
