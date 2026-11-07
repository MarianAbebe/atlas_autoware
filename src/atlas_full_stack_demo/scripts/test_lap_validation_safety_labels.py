#!/usr/bin/env python3
"""Ensure saved safety events name the actual publisher node."""
import unittest
import run_lap_validation as validation


class SafetySourceLabels(unittest.TestCase):
    def test_topics_map_to_guard_and_localization_watchdog(self):
        self.assertEqual(validation.SAFETY_STATUS_SOURCES, {
            '/atlas_loc/safety_status': 'perception_guard',
            '/atlas_loc/base_safety_status': 'localization_watchdog',
        })


if __name__ == '__main__':
    unittest.main()
