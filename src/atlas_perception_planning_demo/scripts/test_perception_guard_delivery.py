#!/usr/bin/env python3
"""Regression tests for isolating authorization delivery from sensor traffic."""
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import perception_guard as guard


class DeliveryIsolation(unittest.TestCase):
    def test_base_callbacks_and_safety_timer_have_independent_groups(self):
        subscriptions=[]
        timers=[]

        class Publisher:
            def publish(self, _):
                pass

        def create_subscription(node, msg_type, topic, callback, qos, *args,
                                callback_group=None, **kwargs):
            subscriptions.append((topic, callback_group))
            return object()

        with patch.object(guard.Node, '__init__', lambda self, name: None), \
             patch.object(guard.Node, 'create_subscription', create_subscription), \
             patch.object(guard.Node, 'create_publisher', lambda *a, **k: Publisher()), \
             patch.object(guard.Node, 'declare_parameter',
                          lambda self, name, default: SimpleNamespace(value=default)), \
             patch.object(guard.Node, 'create_service', lambda *a, **k: object()), \
             patch.object(guard.Node, 'create_timer',
                          lambda self, period, callback, callback_group=None:
                          timers.append(callback_group)):
            node=guard.Guard()

        groups=dict(subscriptions)
        self.assertIs(groups['base_safety_heartbeat'], node.base_callback_group)
        self.assertIs(groups['base_safety_status'], node.base_callback_group)
        self.assertIs(groups['lidar/raw'], node.sensor_callback_group)
        self.assertNotIn('lidar/extended', groups)
        self.assertNotIn('lidar/deskewed', groups)
        self.assertNotIn('lidar/compact', groups)
        self.assertIs(groups['obstacles/pointcloud'], node.sensor_callback_group)
        self.assertIs(groups['obstacles/predicted_objects'], node.predicted_callback_group)
        self.assertIs(groups['obstacles/detected_objects'], node.sensor_callback_group)
        self.assertEqual(timers, [node.timer_callback_group])
        self.assertEqual(len({id(node.base_callback_group), id(node.sensor_callback_group),
                              id(node.predicted_callback_group), id(node.timer_callback_group)}), 4)

    def test_predicted_object_callback_records_receipt_gap_and_runtime(self):
        node=object.__new__(guard.Guard)
        node.state_lock=threading.RLock()
        node.samples={}
        node.predicted_callback_count=0
        node.predicted_callback_last_receipt_monotonic=None
        node.predicted_callback_max_gap_sec=0.0
        node.predicted_callback_last_duration_sec=None
        node.predicted_callback_max_duration_sec=0.0
        node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_200_000_000))
        message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0)))
        with patch.object(guard.time,'monotonic',side_effect=[100.0,100.004,100.1,100.103]):
            node.predicted_objects(message)
            node.predicted_objects(message)
        self.assertEqual(node.predicted_callback_count,2)
        self.assertAlmostEqual(node.samples['predicted_objects'][0],10.0)
        self.assertAlmostEqual(node.samples['predicted_objects'][1],100.1)
        self.assertAlmostEqual(node.samples['predicted_objects'][3],.2)
        self.assertAlmostEqual(node.predicted_callback_max_gap_sec,.1)
        self.assertAlmostEqual(node.predicted_callback_last_duration_sec,.003)

    def test_detected_object_callback_is_diagnostic_only_and_records_gap(self):
        node=object.__new__(guard.Guard)
        node.state_lock=threading.RLock()
        node.detected_callback_count=0
        node.detected_callback_last_receipt_monotonic=None
        node.detected_callback_max_gap_sec=0.0
        node.detected_callback_last_duration_sec=None
        node.detected_callback_max_duration_sec=0.0
        node.detected_callback_last_source_stamp=None
        node.detected_callback_last_source_age_sec=None
        node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_200_000_000))
        message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0)))
        with patch.object(guard.time,'monotonic',side_effect=[100.0,100.002,100.1,100.103]):
            node.detected_objects(message)
            node.detected_objects(message)
        self.assertEqual(node.detected_callback_count,2)
        self.assertAlmostEqual(node.detected_callback_last_source_stamp,10.0)
        self.assertAlmostEqual(node.detected_callback_last_source_age_sec,.2)
        self.assertAlmostEqual(node.detected_callback_max_gap_sec,.1)
        self.assertAlmostEqual(node.detected_callback_last_duration_sec,.003)
        self.assertAlmostEqual(node.detected_callback_max_duration_sec,.003)
        self.assertFalse(hasattr(node,'samples'))

    def test_base_status_callback_records_ros_age_and_monotonic_gap(self):
        node=object.__new__(guard.Guard)
        node.state_lock=threading.RLock()
        node.base=None
        node.samples={}
        node.base_callback_counts={'base_status': 0, 'base_heartbeat': 0}
        node.base_callback_last_receipt_monotonic={}
        node.base_callback_max_gap_sec={}
        node.base_callback_last_duration_sec={}
        node.base_callback_max_duration_sec={}
        node.base_callback_source_age_sec={}
        node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_500_000_000))
        message=SimpleNamespace(data='{"stamp":10.0,"armed":true,"latched":false}')
        with patch.object(guard.time, 'monotonic', side_effect=[100.0,100.004,100.2,100.203]):
            node.base_status(message)
            node.base_status(message)

        self.assertEqual(node.samples['base_status'][:2], (10.0, 100.2))
        self.assertEqual(node.base_callback_counts['base_status'], 2)
        self.assertAlmostEqual(node.base_callback_source_age_sec['base_status'], .5)
        self.assertAlmostEqual(node.base_callback_max_gap_sec['base_status'], .2)
        self.assertAlmostEqual(node.base_callback_last_duration_sec['base_status'], .003)

    def test_freshness_limit_and_authorization_semantics_are_unchanged(self):
        names=('raw','pointcloud','predicted_objects','base_heartbeat','base_status')
        node=SimpleNamespace(state_lock=threading.RLock(), timeout=.3,
            samples={name:(.9,99.9,.02,.1) for name in names},
            base={'armed':True,'latched':False}, input_health={},
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000)))
        with patch.object(guard.time, 'monotonic', return_value=100.0):
            self.assertEqual(guard.Guard.stale(node), [])
            node.samples['base_status']=(.7,99.9,.02,.1)
            self.assertIn('base_status', guard.Guard.stale(node))
            node.samples['base_status']=(.9,99.9,.02,.1)
            node.base['armed']=False
            self.assertIn('base_authorization', guard.Guard.stale(node))

    def test_stale_base_inputs_keep_latch_and_withhold_guard_heartbeat(self):
        import json
        statuses=[];heartbeats=[]
        lock=threading.RLock();publish_lock_state=[]
        node=SimpleNamespace(state_lock=lock, last_tick=None,
            last_tick_duration=None,last_tick_gap=None,max_tick_gap=0.0,tick_count=0,
            stale=lambda:['base_status'], ready=True,bad=False,fault_at=None,
            fault_inputs=[],fault_health=None,fault_tick_gap=None,good_since=99.0,
            input_health={'base_status':{'stamp_age_sec':.301}},
            base_callback_counts={'base_status':3,'base_heartbeat':3},
            base_callback_last_receipt_monotonic={},base_callback_max_gap_sec={},
            base_callback_last_duration_sec={},base_callback_max_duration_sec={},
            base_callback_source_age_sec={},base=None,
            predicted_callback_count=4,predicted_callback_last_receipt_monotonic=99.9,
            predicted_callback_max_gap_sec=.2,predicted_callback_last_duration_sec=.001,
            predicted_callback_max_duration_sec=.002,
            detected_callback_count=0, detected_callback_last_receipt_monotonic=None,
            detected_callback_max_gap_sec=0.0, detected_callback_last_duration_sec=None,
            detected_callback_max_duration_sec=0.0, detected_callback_last_source_stamp=None,
            detected_callback_last_source_age_sec=None,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(
                nanoseconds=10_000_000_000,to_msg=lambda:'stamp')),
            pub=SimpleNamespace(publish=heartbeats.append),
            status=SimpleNamespace(publish=lambda msg: (publish_lock_state.append(lock._is_owned()),
                                                        statuses.append(msg))) )
        with patch.object(guard.time,'monotonic',side_effect=[100.0,100.001]):
            guard.Guard.tick(node)
        report=json.loads(statuses[0].data)
        self.assertTrue(node.bad)
        self.assertEqual(node.fault_inputs,['base_status'])
        self.assertEqual(heartbeats,[])
        self.assertEqual(publish_lock_state,[False])
        self.assertEqual(report['instrumentation']['base_callback_diagnostics']['callback_count']['base_status'],3)
        self.assertEqual(report['instrumentation']['predicted_objects_callback']['count'],4)
        self.assertEqual(report['instrumentation']['detected_objects_callback']['count'],0)
        self.assertEqual(report['fault_inputs'],['base_status'])

    def test_main_uses_four_thread_executor(self):
        node=SimpleNamespace(shutdown_requested=False, destroy_node=lambda:None)
        executors=[]

        class FakeExecutor:
            def __init__(self, num_threads):
                self.num_threads=num_threads
                executors.append(self)
            def add_node(self, value):
                self.node=value
            def spin_once(self, timeout_sec):
                pass
            def shutdown(self):
                pass

        with patch.object(guard.rclpy, 'init'), \
             patch.object(guard, 'Guard', return_value=node), \
             patch.object(guard, 'MultiThreadedExecutor', FakeExecutor), \
             patch.object(guard.rclpy, 'ok', side_effect=[True,False]), \
             patch.object(guard.rclpy, 'try_shutdown'):
            guard.main()
        self.assertEqual(executors[0].num_threads, 4)
        self.assertIs(executors[0].node, node)


if __name__ == '__main__':
    unittest.main()
