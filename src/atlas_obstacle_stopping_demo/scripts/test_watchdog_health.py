#!/usr/bin/env python3
"""Pure decision tests of the actual watchdog methods; no simulator or ROS graph."""
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[3]
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod
base=load('atlas_base_watchdog',ROOT/'src/atlas_closed_loop_demo/scripts/feedback_watchdog.py')
guard=load('atlas_obstacle_guard',ROOT/'src/atlas_obstacle_stopping_demo/scripts/obstacle_guard.py')
NAMES=['odometry','steering','acceleration','trajectory','command']
clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000))

class Decisions(unittest.TestCase):
    def test_guard_isolates_base_callbacks_and_safety_timer(self):
        subscriptions=[];timers=[]
        class Pub:
            def publish(self,_): pass
        def create_subscription(node,typ,topic,callback,depth,*args,callback_group=None,**kwargs):
            subscriptions.append((topic,callback_group));return object()
        with patch.object(guard.Node,'__init__',lambda self,name:None), \
             patch.object(guard.Node,'create_subscription',create_subscription), \
             patch.object(guard.Node,'create_publisher',lambda *a,**k:Pub()), \
             patch.object(guard.Node,'declare_parameter',lambda self,name,default:SimpleNamespace(value=default)), \
             patch.object(guard.Node,'create_service',lambda *a,**k:object()), \
             patch.object(guard.Node,'create_timer',lambda self,period,callback,callback_group=None:timers.append(callback_group)):
            node=guard.Guard()
        groups=dict(subscriptions)
        self.assertIs(groups['base_safety_status'],node.base_callback_group)
        self.assertIs(groups['base_safety_heartbeat'],node.base_callback_group)
        self.assertIs(groups['obstacles/pointcloud'],node.sensor_callback_group)
        self.assertIs(groups['obstacles/predicted_objects'],node.sensor_callback_group)
        self.assertEqual(timers,[node.timer_callback_group])
        self.assertEqual(len({id(node.sensor_callback_group),id(node.base_callback_group),
                              id(node.timer_callback_group)}),3)

    def test_guard_records_base_status_delivery_timing(self):
        node=object.__new__(guard.Guard)
        node.state_lock=threading.RLock()
        node.base=None;node.samples={}
        node.base_callback_counts={'base_status':0,'base_heartbeat':0}
        node.base_callback_last_receipt_monotonic={}
        node.base_callback_max_gap_sec={}
        node.base_callback_last_duration_sec={}
        node.base_callback_max_duration_sec={}
        node.base_callback_source_age_sec={}
        node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_500_000_000))
        msg=SimpleNamespace(data='{"stamp":10.0,"armed":true,"latched":false}')
        with patch.object(guard.time,'monotonic',side_effect=[100.0,100.004,100.2,100.203]):
            guard.Guard.base_status(node,msg)
            guard.Guard.base_status(node,msg)
        self.assertEqual(node.samples['base_status'],(10.0,100.2))
        self.assertEqual(node.base_callback_counts['base_status'],2)
        self.assertAlmostEqual(node.base_callback_source_age_sec['base_status'],.5)
        self.assertAlmostEqual(node.base_callback_max_gap_sec['base_status'],.2)
        self.assertAlmostEqual(node.base_callback_last_duration_sec['base_status'],.003)

    def test_guard_main_uses_three_thread_executor(self):
        node=SimpleNamespace(shutdown_requested=False,destroy_node=lambda:None)
        created=[]
        class FakeExecutor:
            def __init__(self,num_threads): self.num_threads=num_threads;created.append(self)
            def add_node(self,value): self.node=value
            def spin_once(self,timeout_sec): pass
            def shutdown(self): pass
        with patch.object(guard.rclpy,'init'), \
             patch.object(guard,'Guard',return_value=node), \
             patch.object(guard,'MultiThreadedExecutor',FakeExecutor), \
             patch.object(guard.rclpy,'ok',side_effect=[True,False]), \
             patch.object(guard.rclpy,'try_shutdown'):
            guard.main()
        self.assertEqual(created[0].num_threads,3)
        self.assertIs(created[0].node,node)

    def test_base_timer_and_input_callbacks_use_distinct_groups(self):
        subscriptions=[];timers=[]
        class Pub:
            def publish(self,_): pass
        def create_subscription(node,typ,topic,callback,depth,callback_group=None):
            subscriptions.append((topic,callback_group));return object()
        with patch.object(base.Node,'__init__',lambda self,name:None), \
             patch.object(base.Node,'create_subscription',create_subscription), \
             patch.object(base.Node,'create_publisher',lambda *a,**k:Pub()), \
             patch.object(base.Node,'declare_parameter',lambda self,name,default:SimpleNamespace(value=default)), \
             patch.object(base.Node,'create_service',lambda *a,**k:object()), \
             patch.object(base.Node,'create_timer',lambda self,period,callback,callback_group=None:timers.append(callback_group)):
            node=base.Watchdog()
        self.assertTrue(subscriptions)
        self.assertTrue(all(group is node.input_callback_group for _,group in subscriptions))
        self.assertEqual(timers,[node.timer_callback_group])
        self.assertIsNot(node.input_callback_group,node.timer_callback_group)

    def test_stale_base_input_withholds_heartbeat_but_publishes_status(self):
        published=[];statuses=[]
        obj=SimpleNamespace(state_lock=threading.RLock(),last_tick=None,tick_count=0,
            last_tick_start_monotonic=None,last_tick_gap_sec=None,max_tick_gap_sec=0.0,
            last_tick_duration_sec=None,stale=lambda:['ndt'],ready=True,bad=False,
            good_since=99.0,arm_stability=1.0,pub=SimpleNamespace(publish=published.append),
            status=SimpleNamespace(publish=statuses.append),fault_at=None,fault_inputs=[],
            input_health={'ndt':{'stamp_age_sec':.31}},fault_health=None,fault_tick_gap=None,
            stationary_since=None,heartbeat_publish_count=0,last_heartbeat_publish_monotonic=None,
            heartbeat_publish_duration_sec=None,status_publish_count=0,last_status_publish_monotonic=None,
            status_publish_duration_sec=None,publication_diagnostics=lambda start: {'status_start':start},
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)),
            get_logger=lambda:SimpleNamespace(warn=lambda _:None),publish_status=lambda stale:statuses.append(stale))
        with patch.object(base.time,'monotonic',side_effect=[100.0,100.001]):
            base.Watchdog.tick(obj)
        self.assertTrue(obj.bad)
        self.assertEqual(obj.fault_inputs,['ndt'])
        self.assertEqual(published,[])
        self.assertEqual(statuses,[['ndt']])

    def test_base_publisher_timing_diagnostics_are_bounded_and_monotonic(self):
        obj=SimpleNamespace(tick_count=12,last_tick_start_monotonic=9.8,last_tick_gap_sec=.02,
            max_tick_gap_sec=.04,last_tick_duration_sec=.0002,heartbeat_publish_count=10,
            last_heartbeat_publish_monotonic=9.7,heartbeat_publish_duration_sec=.0001,
            status_publish_count=11,last_status_publish_monotonic=9.78,
            last_status_publish_call_start_monotonic=9.781,
            status_publish_duration_sec=.0002,status_publish_operation_duration_sec=.001)
        with patch.object(base.time,'process_time',return_value=2.0):
            values=base.Watchdog.publication_diagnostics(obj,9.9)
        self.assertEqual(values['heartbeat_publish_count'],10)
        self.assertEqual(values['status_publish_start_monotonic_sec'],9.9)
        self.assertEqual(values['status_previous_publish_call_start_monotonic_sec'],9.781)
        self.assertEqual(values['watchdog_tick_max_gap_sec'],.04)
        self.assertGreaterEqual(values['process_cpu_time_sec'],0)

    def test_base_status_publish_does_not_hold_input_state_lock(self):
        lock=threading.RLock();observed=[]
        obj=SimpleNamespace(state_lock=lock,get_clock=clock,ready=True,bad=False,
            fault_at=None,fault_inputs=[],stationary_since=None,fault_health=None,
            fault_tick_gap=None,publication_diagnostics=lambda start:{},
            status_publish_count=0,last_status_publish_monotonic=None,
            last_status_publish_call_start_monotonic=None,
            status_publish_duration_sec=None,status_publish_operation_duration_sec=None)
        def publish(_):
            observed.append(lock._is_owned())
        obj.status=SimpleNamespace(publish=publish)
        with patch.object(base.time,'monotonic',side_effect=[10.0,10.0001,10.001]):
            base.Watchdog.publish_status(obj,[])
        self.assertEqual(observed,[False])
        self.assertGreaterEqual(obj.status_publish_duration_sec,0.0)

    def test_base_age_boundaries_and_missing(self):
        for age,receipt_age,missing,expected in [(.1,.1,False,False),(.299,.299,False,False),
            (.3,.1,False,True),(.1,.3,False,True),(-.001,.1,False,True),
            (.5,.01,False,True),(.1,.1,True,True)]:
            with self.subTest(age=age,receipt=receipt_age,missing=missing):
                obj=SimpleNamespace(stamps={n:1-age for n in NAMES},receipts={n:-receipt_age for n in NAMES},
                                    timeout=.3,get_clock=clock,state_lock=threading.RLock())
                if missing:obj.stamps.pop('odometry');obj.receipts.pop('odometry')
                with patch.object(base.time,'monotonic',return_value=0):
                    stale=base.Watchdog.stale(obj)
                self.assertEqual('odometry' in stale,expected)
                if not missing:self.assertAlmostEqual(obj.input_health['odometry']['stamp_age_sec'],age)
    def test_freeze_stale_despite_new_receipt(self):
        obj=SimpleNamespace(stamps={n:.5 for n in NAMES},receipts={n:100 for n in NAMES},timeout=.3,get_clock=clock,state_lock=threading.RLock())
        with patch.object(base.time,'monotonic',return_value=100):
            self.assertEqual(base.Watchdog.stale(obj),NAMES)
    def test_health_snapshot_not_rewritten_after_recovery(self):
        obj=SimpleNamespace(stamps={n:.5 for n in NAMES},receipts={n:99 for n in NAMES},timeout=.3,get_clock=clock,state_lock=threading.RLock())
        with patch.object(base.time,'monotonic',return_value=100):base.Watchdog.stale(obj)
        snapshot=obj.input_health
        obj.stamps={n:.99 for n in NAMES};obj.receipts={n:100 for n in NAMES}
        with patch.object(base.time,'monotonic',return_value=100):self.assertEqual(base.Watchdog.stale(obj),[])
        self.assertEqual(snapshot['command']['stamp_age_sec'],.5)
    def test_guard_authorization_and_both_age_limits(self):
        names=['pointcloud','predicted_objects','base_heartbeat','base_status']
        for age,receipt_age,authorized in [(.1,.1,True),(.4,.1,True),(.1,.4,True),(.1,.1,False)]:
            obj=SimpleNamespace(samples={n:(1-age,100-receipt_age) for n in names},
                base=dict(armed=authorized,latched=not authorized),timeout=.3,get_clock=clock,
                state_lock=threading.RLock())
            with patch.object(guard.time,'monotonic',return_value=100):bad=guard.Guard.stale(obj)
            self.assertEqual('pointcloud' in bad,age>=.3 or receipt_age>=.3)
            self.assertEqual('base_authorization' in bad,not authorized)

    def test_guard_status_preserves_base_publisher_timing(self):
        import json
        published=[]
        base_state={'stamp':10.0,'armed':True,'latched':False,'fault_inputs':[],
                    'instrumentation':{'publisher_timing':{'heartbeat_publish_count':42}}}
        obj=SimpleNamespace(stale=lambda:[],get_clock=clock,ready=False,bad=False,
            good_since=99.5,fault_at=None,fault_inputs=[],fault_health=None,
            base=base_state,input_health={'base_status':{'receipt_age_sec':.01}},
            state_lock=threading.RLock(),base_callback_counts={},
            base_callback_last_receipt_monotonic={},base_callback_max_gap_sec={},
            base_callback_last_duration_sec={},base_callback_max_duration_sec={},
            base_callback_source_age_sec={},
            status=SimpleNamespace(publish=published.append),
            pub=SimpleNamespace(publish=lambda _:None))
        with patch.object(guard.time,'monotonic',return_value=100):guard.Guard.tick(obj)
        state=json.loads(published[0].data)
        self.assertEqual(state['base_watchdog_state']['instrumentation']['publisher_timing']['heartbeat_publish_count'],42)
        self.assertEqual(state['current_input_health']['base_status']['receipt_age_sec'],.01)

if __name__=='__main__':unittest.main()
