#!/usr/bin/env python3
"""Deterministic tests for the diagnostic Humble executor adapter (no ROS graph)."""
import unittest
from types import SimpleNamespace
from watchdog_topic_probe import TimingExecutor


class Handle:
    def __init__(self, result):
        self.result = result

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def take_message(self, typ, raw):
        return self.result


class TimingExecutorTest(unittest.TestCase):
    def test_metadata_retained_without_changing_message(self):
        message = object()
        metadata = {'source_timestamp': 100, 'received_timestamp': 110}
        sub = SimpleNamespace(handle=Handle((message, metadata)), msg_type=object, raw=False)
        executor = SimpleNamespace()
        self.assertIs(TimingExecutor._take_subscription(executor, sub), message)
        self.assertIs(executor.message_info, metadata)

    def test_empty_take_does_not_dispatch(self):
        executor = SimpleNamespace()
        sub = SimpleNamespace(handle=Handle(None), msg_type=object, raw=False)
        self.assertIsNone(TimingExecutor._take_subscription(executor, sub))


if __name__ == '__main__':
    unittest.main()
