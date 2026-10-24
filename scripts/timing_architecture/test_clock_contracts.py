#!/usr/bin/env python3
"""Deterministic clock-domain cases using the unchanged Atlas watchdog predicate."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('actual_watchdog',ROOT/'src/atlas_closed_loop_demo/scripts/feedback_watchdog.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
NAMES=['odometry','steering','acceleration','trajectory','command']


def monitor(ros_now,source,receipt):
    return SimpleNamespace(stamps={n:source for n in NAMES},receipts={n:receipt for n in NAMES},timeout=.3,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(ros_now*1e9))))


class ClockContracts(unittest.TestCase):
    def test_finite_old_message_is_stale_even_after_fresh_callback(self):
        obj=monitor(1.31,1.,100.)
        with patch.object(base.time,'monotonic',return_value=100.):
            self.assertEqual(base.Watchdog.stale(obj),NAMES)
        self.assertAlmostEqual(obj.input_health['command']['receipt_age_sec'],0.)

    def test_paused_ros_clock_does_not_pause_receipt_safety(self):
        obj=monitor(1.,1.,100.)
        with patch.object(base.time,'monotonic',return_value=100.):
            self.assertEqual(base.Watchdog.stale(obj),[])
        with patch.object(base.time,'monotonic',return_value=100.5):
            self.assertEqual(base.Watchdog.stale(obj),NAMES)
        self.assertEqual(obj.input_health['command']['stamp_age_sec'],0.)

    def test_timer_before_ready_subscription_can_observe_old_state(self):
        obj=monitor(1.31,1.,100.)
        with patch.object(base.time,'monotonic',return_value=100.01):
            self.assertEqual(base.Watchdog.stale(obj),NAMES)
        # A ready DDS sample cannot update application state until its callback runs.
        for n in NAMES:
            obj.stamps[n]=1.30;obj.receipts[n]=100.01
        with patch.object(base.time,'monotonic',return_value=100.01):
            self.assertEqual(base.Watchdog.stale(obj),[])

    def test_delayed_simulation_time_itself_must_still_expire(self):
        obj=monitor(.31,0.,100.)
        with patch.object(base.time,'monotonic',return_value=100.):
            self.assertEqual(base.Watchdog.stale(obj),NAMES)


if __name__=='__main__':unittest.main()
