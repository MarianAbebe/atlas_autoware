#!/usr/bin/env python3
"""Regression for preserving object-adapter timing payloads in lap evidence."""
import unittest

from run_lap_validation import build_launch_command, object_adapter_timing_record


class AdapterTimingCapture(unittest.TestCase):
    def test_headless_launch_disables_marker_unless_explicitly_requested(self):
        default = build_launch_command(5, '/tmp/atlas-run')
        self.assertIn('launch_rviz:=false', default)
        self.assertIn('launch_ego_marker:=false', default)
        self.assertIn('steering_step_diagnostics:=false', default)
        self.assertIn('launch_ego_marker:=true', build_launch_command(1, '/tmp/atlas-one', True))
        diagnostic = build_launch_command(1, '/tmp/atlas-diag', steering_step_diagnostics=True)
        self.assertIn('steering_step_diagnostics:=true', diagnostic)

    def test_raw_adapter_payload_and_collector_clock_are_both_preserved(self):
        payload = '{"event":"predicted_objects_published","header_stamp":42.0,' \
                  '"detected_to_publish_sec":0.002}'
        receipt = 1234.5
        row = object_adapter_timing_record(payload, receipt)
        self.assertEqual(row, {
            'collector_receipt_monotonic_sec': receipt,
            'payload': payload,
        })


if __name__ == '__main__':
    unittest.main()
