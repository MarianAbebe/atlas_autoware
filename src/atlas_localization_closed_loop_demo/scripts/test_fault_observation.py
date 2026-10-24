#!/usr/bin/env python3
"""Regression for first-latch ordering versus eventual injected-input staleness."""
import unittest
from run_validation import expected_input_became_stale


class Observation(unittest.TestCase):
    def test_ndt_can_expire_before_ekf_odometry(self):
        statuses=[dict(t=5.29,currently_stale=['ndt']),
                  dict(t=5.32,currently_stale=['ndt','odometry'])]
        self.assertTrue(expected_input_became_stale(statuses,5.,'odometry'))

    def test_another_fault_is_not_sufficient(self):
        self.assertFalse(expected_input_became_stale([dict(t=5.29,currently_stale=['ndt'])],5.,'odometry'))

    def test_late_or_preinjection_observation_is_not_sufficient(self):
        for t in [4.9,5.41]:
            self.assertFalse(expected_input_became_stale([dict(t=t,currently_stale=['odometry'])],5.,'odometry'))

    def test_missing_injection_is_not_sufficient(self):
        self.assertFalse(expected_input_became_stale([],None,'odometry'))


if __name__ == '__main__':
    unittest.main()
