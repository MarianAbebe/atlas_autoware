#!/usr/bin/env python3
"""Controlled reader-backlog experiment; does not establish the archived cause.

Withhold raw-observer takes while servicing the real Atlas relay forwarding
method and its output reader. KEEP_LAST can then lose raw observations despite
successful command delivery. No watchdog or controller is run or modified.
Use an isolated ROS_DOMAIN_ID, after the driving suite has ended.
"""
import sys
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
import rclpy
from autoware_control_msgs.msg import Control

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/atlas_closed_loop_demo/scripts'))
spec = importlib.util.spec_from_file_location('atlas_relay', ROOT / 'src/atlas_closed_loop_demo/scripts/feedback_relay.py')
relay_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay_module)


def take(subscription, timeout=3):
    deadline = time.monotonic() + timeout
    while True:
        with subscription.handle:
            result = subscription.handle.take_message(subscription.msg_type, subscription.raw)
        if result is not None:
            return result
        if time.monotonic() >= deadline:
            raise AssertionError('timed out waiting for middleware sample')
        time.sleep(.001)


class ObservationTest(unittest.TestCase):
    def test_withheld_raw_reader_can_miss_delivered_commands(self):
        rclpy.init()
        node = rclpy.create_node('command_observation_negative_control')
        try:
            raw_pub = node.create_publisher(Control, 'observation_test/raw', 10)
            output_pub = node.create_publisher(Control, 'observation_test/output', 10)
            relay_sub = node.create_subscription(Control, 'observation_test/raw', lambda m: None, 10)
            raw_observer = node.create_subscription(Control, 'observation_test/raw', lambda m: None, 10)
            delivered_sub = node.create_subscription(Control, 'observation_test/output', lambda m: None, 10)
            deadline = time.monotonic() + 5
            while raw_pub.get_subscription_count() != 2 or output_pub.get_subscription_count() != 1:
                self.assertLess(time.monotonic(), deadline, 'discovery timeout')
                time.sleep(.01)
            relay = SimpleNamespace(originals={}, inject='live', fault_input='odometry')
            delivered = []
            for sequence in range(1, 31):
                command = Control()
                command.stamp.sec = sequence
                raw_pub.publish(command)
                forwarded, _ = take(relay_sub)
                relay_module.Relay.forward(relay, 'command', forwarded, output_pub)
                output, _ = take(delivered_sub)
                delivered.append(output.stamp.sec)
            observed = []
            for _ in range(10):
                message, info = take(raw_observer)
                observed.append(message.stamp.sec)
                self.assertIn('received_timestamp', info)
            with raw_observer.handle:
                self.assertIsNone(raw_observer.handle.take_message(Control, False))
            self.assertEqual(delivered, list(range(1, 31)))
            self.assertEqual(observed, list(range(21, 31)))
            print('Controlled backlog: published=30 relay_delivered=30 raw_observed=10; no shutdown involved')
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
