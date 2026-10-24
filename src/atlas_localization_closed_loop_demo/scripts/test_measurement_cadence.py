#!/usr/bin/env python3
"""Deterministic regression for repeated work in the old boundary window."""
import unittest
from measurement_cadence import MeasurementCadence


class Cadence(unittest.TestCase):
    def test_boundary_burst_samples_once(self):
        times = [.500+i*.001 for i in range(20)]
        old = [t for t in times if int(t*2) != int((t-.02)*2)]
        self.assertGreater(len(old), 1)
        cadence = MeasurementCadence()
        self.assertEqual(sum(cadence.due(t) for t in times), 1)

    def test_delay_skips_backlog(self):
        cadence = MeasurementCadence()
        self.assertFalse(cadence.due(.49))
        self.assertTrue(cadence.due(.5))
        self.assertTrue(cadence.due(3.7))
        self.assertFalse(cadence.due(3.7))
        self.assertFalse(cadence.due(4.19))
        self.assertTrue(cadence.due(4.2))

    def test_exact_interval_and_invalid_interval(self):
        cadence = MeasurementCadence()
        self.assertEqual([cadence.due(t) for t in [0,.5,1,1.5]], [False,True,True,True])
        with self.assertRaises(ValueError):
            MeasurementCadence(0)

if __name__ == '__main__':
    unittest.main()
