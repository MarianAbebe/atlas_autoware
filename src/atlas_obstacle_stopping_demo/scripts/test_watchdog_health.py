#!/usr/bin/env python3
"""Pure decision tests of the actual watchdog methods; no simulator or ROS graph."""
import importlib.util
from pathlib import Path
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
    def test_base_age_boundaries_and_missing(self):
        for age,receipt_age,missing,expected in [(.1,.1,False,False),(.299,.299,False,False),
            (.3,.1,False,True),(.1,.3,False,True),(-.001,.1,False,True),
            (.5,.01,False,True),(.1,.1,True,True)]:
            with self.subTest(age=age,receipt=receipt_age,missing=missing):
                obj=SimpleNamespace(stamps={n:1-age for n in NAMES},receipts={n:-receipt_age for n in NAMES},
                                    timeout=.3,get_clock=clock)
                if missing:obj.stamps.pop('odometry');obj.receipts.pop('odometry')
                with patch.object(base.time,'monotonic',return_value=0):
                    stale=base.Watchdog.stale(obj)
                self.assertEqual('odometry' in stale,expected)
                if not missing:self.assertAlmostEqual(obj.input_health['odometry']['stamp_age_sec'],age)
    def test_freeze_stale_despite_new_receipt(self):
        obj=SimpleNamespace(stamps={n:.5 for n in NAMES},receipts={n:100 for n in NAMES},timeout=.3,get_clock=clock)
        with patch.object(base.time,'monotonic',return_value=100):
            self.assertEqual(base.Watchdog.stale(obj),NAMES)
    def test_health_snapshot_not_rewritten_after_recovery(self):
        obj=SimpleNamespace(stamps={n:.5 for n in NAMES},receipts={n:99 for n in NAMES},timeout=.3,get_clock=clock)
        with patch.object(base.time,'monotonic',return_value=100):base.Watchdog.stale(obj)
        snapshot=obj.input_health
        obj.stamps={n:.99 for n in NAMES};obj.receipts={n:100 for n in NAMES}
        with patch.object(base.time,'monotonic',return_value=100):self.assertEqual(base.Watchdog.stale(obj),[])
        self.assertEqual(snapshot['command']['stamp_age_sec'],.5)
    def test_guard_authorization_and_both_age_limits(self):
        names=['pointcloud','predicted_objects','base_heartbeat','base_status']
        for age,receipt_age,authorized in [(.1,.1,True),(.4,.1,True),(.1,.4,True),(.1,.1,False)]:
            obj=SimpleNamespace(samples={n:(1-age,100-receipt_age) for n in names},
                base=dict(armed=authorized,latched=not authorized),timeout=.3,get_clock=clock)
            with patch.object(guard.time,'monotonic',return_value=100):bad=guard.Guard.stale(obj)
            self.assertEqual('pointcloud' in bad,age>=.3 or receipt_age>=.3)
            self.assertEqual('base_authorization' in bad,not authorized)

if __name__=='__main__':unittest.main()
