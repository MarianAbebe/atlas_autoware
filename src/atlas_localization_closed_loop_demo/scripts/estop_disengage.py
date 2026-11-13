#!/usr/bin/env python3
"""Disengage the simulated Autoware gate on E-stop; never issue auto-reengage."""
import json

import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import Engage
from std_msgs.msg import String
from tier4_external_api_msgs.msg import Emergency


class EstopDisengage(Node):
    def __init__(self):
        super().__init__('atlas_estop_disengage')
        self.emergency_active = False
        self.disengage_pending = False
        self.gate_engaged = None
        self.published_disengage_count = 0
        self.command = self.create_publisher(Engage, '/atlas_loc/gate/input/engage', 1)
        self.status = self.create_publisher(String, '/atlas_loc/estop_disengage/status', 10)
        self.create_subscription(Emergency, '/atlas_loc/gate/output/external_emergency',
                                 self.on_emergency, 10)
        self.create_subscription(Engage, '/atlas_loc/gate/output/engage',
                                 self.on_engage, 10)
        self.create_timer(0.05, self.tick)

    def on_emergency(self, msg):
        self.emergency_active = bool(msg.emergency)
        if self.emergency_active:
            self.disengage_pending = True

    def on_engage(self, msg):
        self.gate_engaged = bool(msg.engage)
        if self.disengage_pending and not self.gate_engaged:
            self.disengage_pending = False

    def tick(self):
        if self.disengage_pending and self.gate_engaged is not False:
            msg = Engage()
            msg.stamp = self.get_clock().now().to_msg()
            msg.engage = False
            self.command.publish(msg)
            self.published_disengage_count += 1
        status = String()
        status.data = json.dumps({
            'emergency_active': self.emergency_active,
            'disengage_pending': self.disengage_pending,
            'gate_engaged': self.gate_engaged,
            'disengage_publish_count': self.published_disengage_count,
        }, separators=(',', ':'))
        self.status.publish(status)


def main():
    rclpy.init()
    node = EstopDisengage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
