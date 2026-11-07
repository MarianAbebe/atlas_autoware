#!/usr/bin/env python3
"""Decision regression: current EKF predictions cannot conceal lost NDT input."""
from collections import defaultdict
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import localization_watchdog as guard


class Freshness(unittest.TestCase):
    def monitor(self):
        node = object.__new__(guard.LocalizationWatchdog)
        node.stamps = {n:.99 for n in ['odometry','steering','acceleration','trajectory','command','ndt']}
        node.receipts = {n:99.99 for n in node.stamps}
        node.timeout = .3
        node.callback_counts = defaultdict(int)
        node.callback_gaps = {}
        node.callback_last_receipt = {}
        node.callback_last_source_age = {}
        node.callback_max_duration = {}
        node.callback_last_duration = {}
        node.state_lock = threading.RLock()
        node.tick_count = 0
        node.last_tick_start_monotonic = None
        node.last_tick_gap_sec = None
        node.max_tick_gap_sec = 0.0
        node.last_tick_duration_sec = None
        node.heartbeat_publish_count = 0
        node.last_heartbeat_publish_monotonic = None
        node.heartbeat_publish_duration_sec = None
        node.status_publish_count = 0
        node.last_status_publish_monotonic = None
        node.last_status_publish_call_start_monotonic = None
        node.status_publish_duration_sec = None
        node.status_publish_operation_duration_sec = None
        node.ndt_runtime_count=0
        node.ndt_runtime_last_receipt_monotonic=None
        node.ndt_runtime_max_receipt_gap_sec=0.0
        node.ndt_runtime_last_source_stamp=None
        node.ndt_runtime_last_source_age_sec=None
        node.ndt_runtime_last_duration_sec=None
        node.ndt_runtime_max_duration_sec=0.0
        node.ndt_runtime_last_ms=None
        node.tick_gap_sec = .02
        node._diagnostic_tick_start = 99.98
        node.get_clock = lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000))
        return node

    def test_current_ekf_does_not_mask_missing_ndt(self):
        node = self.monitor()
        del node.stamps['ndt']
        with patch.object(guard.time,'monotonic',return_value=100):
            self.assertEqual(node.stale(),['ndt'])

    def test_ndt_source_and_receipt_limits(self):
        for source,receipt in [(.69,99.99),(.99,99.69),(.5,100)]:
            node = self.monitor()
            node.stamps['ndt']=source;node.receipts['ndt']=receipt
            with patch.object(guard.time,'monotonic',return_value=100):
                self.assertEqual(node.stale(),['ndt'])

    def test_exact_source_boundary_is_stale(self):
        node = self.monitor()
        node.get_clock = lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=0))
        node.stamps = {n:-.01 for n in node.stamps}
        node.receipts = {n:-.01 for n in node.receipts}
        node.stamps['ndt']=-.3
        with patch.object(guard.time,'monotonic',return_value=0):
            self.assertEqual(node.stale(),['ndt'])
        node.stamps['ndt']=-.01
        node.receipts['ndt']=-.3
        with patch.object(guard.time,'monotonic',return_value=0):
            self.assertEqual(node.stale(),['ndt'])

    def test_receive_records_input_callback_timing_without_changing_stamp(self):
        node = self.monitor()
        msg = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=10, nanosec=0)))
        node.get_clock = lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10200000000))
        with patch.object(guard.time, 'monotonic', side_effect=[100.0, 100.0, 100.0, 100.1, 100.1, 100.1]):
            node.receive('ndt', msg)
            node.receive('ndt', msg)
        self.assertEqual(node.callback_counts['ndt'], 2)
        self.assertAlmostEqual(node.callback_gaps['ndt'], .1)
        self.assertAlmostEqual(node.callback_last_source_age['ndt'], .2)
        self.assertGreaterEqual(node.callback_last_duration['ndt'], 0.0)
        self.assertGreaterEqual(node.callback_max_duration['ndt'], node.callback_last_duration['ndt'])
        self.assertEqual(node.stamps['ndt'], 10.0)
        self.assertEqual(node.receipts['ndt'], 100.1)

    def test_ndt_runtime_observer_records_timing_without_touching_freshness(self):
        node = self.monitor()
        node.ndt_runtime_count=0
        node.ndt_runtime_last_receipt_monotonic=None
        node.ndt_runtime_max_receipt_gap_sec=0.0
        node.ndt_runtime_last_source_stamp=None
        node.ndt_runtime_last_source_age_sec=None
        node.ndt_runtime_last_duration_sec=None
        node.ndt_runtime_max_duration_sec=0.0
        node.ndt_runtime_last_ms=None
        node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000))
        before=(dict(node.stamps),dict(node.receipts))
        msg=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0),data=12.5)
        with patch.object(guard.time,'monotonic',side_effect=[100.0,100.002,100.1,100.103]):
            node.ndt_runtime(msg)
            node.ndt_runtime(msg)
        self.assertEqual(node.ndt_runtime_count,2)
        self.assertAlmostEqual(node.ndt_runtime_max_receipt_gap_sec,.1)
        self.assertAlmostEqual(node.ndt_runtime_last_source_age_sec,.0)
        self.assertEqual(node.ndt_runtime_last_ms,12.5)
        self.assertEqual((node.stamps,node.receipts),before)

    def test_status_includes_bounded_input_and_tick_diagnostics(self):
        import json
        node = self.monitor()
        published = []
        publish_lock_state=[]
        node.status = SimpleNamespace(publish=lambda msg: (publish_lock_state.append(node.state_lock._is_owned()),
                                                           published.append(msg)))
        node.ready = False; node.bad = False; node.fault_at = None
        node.fault_inputs = []; node.stationary_since = None
        node.fault_health = None; node.fault_tick_gap = None
        node.callback_counts['ndt'] = 3
        node.callback_gaps['ndt'] = .12
        node.callback_last_receipt['ndt'] = 99.9
        node.callback_last_source_age['ndt'] = .08
        node.callback_last_duration['ndt'] = .001
        node.callback_max_duration['ndt'] = .002
        node.tick_count=12;node.last_tick_start_monotonic=99.98
        node.max_tick_gap_sec=.03;node.last_tick_duration_sec=.0005
        node.heartbeat_publish_count=9;node.last_heartbeat_publish_monotonic=99.9
        node.heartbeat_publish_duration_sec=.0001
        node.status_publish_count=11;node.last_status_publish_monotonic=99.96
        node.status_publish_duration_sec=.0002
        node.publication_diagnostics=guard.existing.Watchdog.publication_diagnostics.__get__(node)
        with patch.object(guard.LocalizationWatchdog, 'get_clock',
                          return_value=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100000000000))):
            node.publish_status([])
        state = json.loads(published[0].data)
        self.assertEqual(state['instrumentation']['input_callback_counts']['ndt'], 3)
        self.assertEqual(state['instrumentation']['input_callback_max_gap_sec']['ndt'], .12)
        self.assertEqual(state['instrumentation']['watchdog_tick_gap_sec'], .02)
        self.assertEqual(state['instrumentation']['input_callback_max_duration_sec']['ndt'], .002)
        self.assertEqual(state['instrumentation']['publisher_timing']['heartbeat_publish_count'], 9)
        self.assertEqual(state['instrumentation']['publisher_timing']['status_publish_count'], 12)
        self.assertEqual(publish_lock_state,[False])
        self.assertEqual(node.status_publish_count,12)
        self.assertIsNotNone(node.last_status_publish_monotonic)
        self.assertIsNotNone(node.last_status_publish_call_start_monotonic)
        self.assertGreaterEqual(node.status_publish_duration_sec,0.0)
        self.assertGreaterEqual(node.status_publish_operation_duration_sec,
                                node.status_publish_duration_sec)

    def test_latch_diagnostic_preserves_source_and_receipt_age(self):
        import json
        health = {'trajectory': {'source_stamp': 12.5, 'stamp_age_sec': .301,
                                 'receipt_age_sec': .302}}
        detail = json.loads(guard.existing.format_fault_detail(health, .021))
        self.assertEqual(detail['fault_input_health'], health)
        self.assertEqual(detail['fault_tick_gap_sec'], .021)

    def test_existing_checks_remain_required(self):
        node = self.monitor()
        node.stamps['command']=.5
        with patch.object(guard.time,'monotonic',return_value=100):
            self.assertEqual(node.stale(),['command'])
        node.stamps['command']=.99
        with patch.object(guard.time,'monotonic',return_value=100):
            self.assertEqual(node.stale(),[])

    def test_shutdown_runtime_error_is_handled_only_after_context_stops(self):
        node = SimpleNamespace(shutdown_requested=False, destroy_node=lambda:None)
        created=[]
        class FakeExecutor:
            def __init__(self, num_threads): self.num_threads=num_threads;self.added=None;created.append(self)
            def add_node(self, value): self.added=value
            def spin_once(self, timeout_sec): raise RuntimeError('context stopped')
            def shutdown(self): return True
        with patch.object(guard.rclpy, 'init'), \
             patch.object(guard, 'LocalizationWatchdog', return_value=node), \
             patch.object(guard, 'MultiThreadedExecutor', FakeExecutor), \
             patch.object(guard.rclpy, 'ok', side_effect=[True, False]), \
             patch.object(guard.rclpy, 'try_shutdown'):
            guard.main()
        self.assertEqual(created[-1].num_threads,3)

        class FailingExecutor(FakeExecutor):
            def spin_once(self, timeout_sec): raise RuntimeError('live failure')
        with patch.object(guard.rclpy, 'init'), \
             patch.object(guard, 'LocalizationWatchdog', return_value=node), \
             patch.object(guard, 'MultiThreadedExecutor', FailingExecutor), \
             patch.object(guard.rclpy, 'ok', return_value=True), \
             patch.object(guard.rclpy, 'try_shutdown'):
            with self.assertRaisesRegex(RuntimeError, 'live failure'):
                guard.main()


if __name__ == '__main__':
    unittest.main()
