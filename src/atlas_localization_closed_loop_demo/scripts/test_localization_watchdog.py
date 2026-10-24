#!/usr/bin/env python3
"""Decision regression: current EKF predictions cannot conceal lost NDT input."""
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

    def test_existing_checks_remain_required(self):
        node = self.monitor()
        node.stamps['command']=.5
        with patch.object(guard.time,'monotonic',return_value=100):
            self.assertEqual(node.stale(),['command'])
        node.stamps['command']=.99
        with patch.object(guard.time,'monotonic',return_value=100):
            self.assertEqual(node.stale(),[])


if __name__ == '__main__':
    unittest.main()
