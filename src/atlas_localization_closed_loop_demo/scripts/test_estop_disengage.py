#!/usr/bin/env python3
"""Deterministic tests for the simulation E-stop disengagement latch."""
import unittest
from builtin_interfaces.msg import Time
from types import SimpleNamespace
from unittest.mock import patch

import estop_disengage as module


class Publisher:
    def __init__(self):
        self.messages = []
    def publish(self, msg):
        self.messages.append(msg)


class DisengageLatchTests(unittest.TestCase):
    def node(self):
        node = object.__new__(module.EstopDisengage)
        node.emergency_active = False
        node.disengage_pending = False
        node.gate_engaged = True
        node.published_disengage_count = 0
        node.command = Publisher()
        node.status = Publisher()
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
        return node

    def test_estop_publishes_only_disengage_until_gate_reports_disarmed(self):
        node = self.node()
        module.EstopDisengage.on_emergency(node, SimpleNamespace(emergency=True))
        module.EstopDisengage.tick(node)
        self.assertEqual(len(node.command.messages), 1)
        self.assertFalse(node.command.messages[0].engage)
        self.assertTrue(node.disengage_pending)
        module.EstopDisengage.on_engage(node, SimpleNamespace(engage=False))
        self.assertFalse(node.disengage_pending)
        module.EstopDisengage.tick(node)
        self.assertEqual(len(node.command.messages), 1)
        self.assertTrue(all(not msg.engage for msg in node.command.messages))

    def test_normal_operation_does_not_issue_engagement(self):
        node = self.node()
        module.EstopDisengage.tick(node)
        self.assertEqual(node.command.messages, [])
        module.EstopDisengage.on_emergency(node, SimpleNamespace(emergency=False))
        self.assertEqual(node.command.messages, [])


if __name__ == '__main__':
    unittest.main()
