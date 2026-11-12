"""Isolated ideal Ackermann bicycle simulator; never a physical actuator interface."""
import json
import math
import time

import rclpy
from autoware_control_msgs.msg import Control
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .ackermann_kinematics import AckermannState, step
from .world import clearance, rectangles


class Simulator(Node):
    def __init__(self):
        super().__init__('ackermann_simulator')
        parameter = lambda name, default: self.declare_parameter(name, default).value
        self.wheelbase = float(parameter('wheelbase_m', 1.2))
        self.steering_limit = float(parameter('steering_limit_rad', 0.5))
        self.steering_rate_limit = float(parameter('steering_rate_limit_radps', 0.6))
        self.acceleration_limit = float(parameter('acceleration_limit_mps2', 0.8))
        self.deceleration_limit = float(parameter('deceleration_limit_mps2', 1.2))
        self.speed_limit = float(parameter('speed_limit_mps', 0.8))
        self.command_timeout = float(parameter('command_timeout_sec', 0.25))
        self.scenario = parameter('scenario', 'open')
        self.robot_radius = float(parameter('robot_radius', 0.32))
        self.initial_x = float(parameter('initial_x_m', -1.0))
        self.initial_y = float(parameter('initial_y_m', 0.0))
        rectangles(self.scenario)

        self.state = AckermannState(x=self.initial_x, y=self.initial_y)
        self.target = (0.0, 0.0)
        self.last_command_mono = None
        self.last_command_mono_ns = None
        self.last_tick_mono = time.monotonic()
        self.sim_time_sec = 0.0
        self.tick_count = 0
        self.encoder_position = 0.0
        self.odometry_enabled = True

        self.odom_pub = self.create_publisher(Odometry, '/atlas_ackermann/odom', 20)
        self.encoder_pub = self.create_publisher(
            JointState, '/atlas_ackermann/encoder_states', 20)
        self.dynamics_pub = self.create_publisher(
            String, '/atlas_ackermann/simulator_dynamics', 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(Control, '/atlas_ackermann/control_cmd',
                                 self.on_command, 10)
        self.create_service(
            SetBool, '~/set_odometry_enabled', self.on_odometry_enabled)
        self.publish_static_transforms()
        self.create_timer(0.02, self.tick)

    def publish_static_transforms(self):
        stamp = self.get_clock().now().to_msg()
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.rotation.w = 1.0
        base_to_laser = TransformStamped()
        base_to_laser.header.stamp = stamp
        base_to_laser.header.frame_id = 'base_link'
        base_to_laser.child_frame_id = 'laser'
        base_to_laser.transform.translation.x = 0.2
        base_to_laser.transform.translation.z = 0.25
        base_to_laser.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform([map_to_odom, base_to_laser])

    def on_odometry_enabled(self, request, response):
        self.odometry_enabled = bool(request.data)
        response.success = True
        response.message = ('odometry and dynamic TF publishing enabled' if request.data
                            else 'odometry and dynamic TF publishing paused')
        return response

    def on_command(self, message):
        values = (message.longitudinal.velocity,
                  message.lateral.steering_tire_angle)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Rejected invalid Ackermann command')
            return
        self.target = values
        self.last_command_mono_ns = time.monotonic_ns()
        self.last_command_mono = self.last_command_mono_ns * 1.0e-9

    def tick(self):
        tick_start = time.monotonic()
        tick_start_ns = int(tick_start * 1.0e9)
        dt = min(0.05, max(0.0, tick_start - self.last_tick_mono))
        self.last_tick_mono = tick_start
        previous = self.state
        command_age = (math.inf if self.last_command_mono is None else
                       tick_start - self.last_command_mono)
        target = self.target if command_age <= self.command_timeout else (0.0, 0.0)
        candidate, yaw_rate = step(
            previous, target[0], target[1], dt, self.wheelbase,
            self.steering_limit, self.steering_rate_limit,
            self.acceleration_limit, self.deceleration_limit, self.speed_limit)
        collision_clamp = clearance(
            candidate.x, candidate.y, self.robot_radius, self.scenario) < 0.0
        if collision_clamp:
            candidate = AckermannState(
                previous.x, previous.y, previous.yaw, 0.0, candidate.steering)
            yaw_rate = 0.0
        self.state = candidate
        self.sim_time_sec += dt
        self.tick_count += 1
        self.encoder_position += self.state.speed * dt

        # Dynamics and pose share this simulation-time sample. This is the
        # authoritative timebase for runtime acceleration measurements; ROS
        # message header and recorder receipt times remain separate diagnostics.
        acceleration = ((self.state.speed - previous.speed) / dt if dt > 0.0 else 0.0)
        steering_rate = ((self.state.steering - previous.steering) / dt
                         if dt > 0.0 else 0.0)
        stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.state.x, self.state.y, self.state.yaw
        if self.odometry_enabled:
            odometry = Odometry()
            odometry.header.stamp = stamp
            odometry.header.frame_id = 'odom'
            odometry.child_frame_id = 'base_link'
            odometry.pose.pose.position.x = x
            odometry.pose.pose.position.y = y
            odometry.pose.pose.orientation.z = math.sin(yaw / 2.0)
            odometry.pose.pose.orientation.w = math.cos(yaw / 2.0)
            odometry.twist.twist.linear.x = self.state.speed
            odometry.twist.twist.angular.z = yaw_rate
            self.odom_pub.publish(odometry)

            transform = TransformStamped()
            transform.header = odometry.header
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation = odometry.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

            encoder = JointState()
            encoder.header.stamp = stamp
            encoder.name = ['rear_axle_encoder']
            encoder.position = [self.encoder_position]
            encoder.velocity = [self.state.speed]
            self.encoder_pub.publish(encoder)

        diagnostic = String()
        diagnostic.data = json.dumps({
            'tick': self.tick_count,
            'tick_monotonic_ns': tick_start_ns,
            'control_receipt_monotonic_ns': self.last_command_mono_ns,
            'simulation_time_sec': self.sim_time_sec,
            'integration_dt_sec': dt,
            'speed_before_mps': previous.speed,
            'speed_after_mps': self.state.speed,
            'acceleration_mps2': acceleration,
            'steering_before_rad': previous.steering,
            'steering_after_rad': self.state.steering,
            'steering_rate_radps': steering_rate,
            'x_m': x, 'y_m': y, 'yaw_rad': yaw,
            'target_speed_mps': target[0],
            'target_steering_rad': target[1],
            'command_age_sec': command_age if math.isfinite(command_age) else None,
            'odometry_enabled': self.odometry_enabled,
            'collision_clamp': collision_clamp,
        }, separators=(',', ':'), allow_nan=False)
        self.dynamics_pub.publish(diagnostic)


def main():
    rclpy.init()
    node = Simulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
