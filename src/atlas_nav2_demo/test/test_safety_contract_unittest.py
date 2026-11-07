"""Dependency-free safety-contract regression tests for the Nav2 fixture."""
import math
import unittest

from atlas_nav2_demo.gate_logic import gated_velocity
from atlas_nav2_demo.kinematics import mix_twist, wheels_to_twist


class SafetyContractTest(unittest.TestCase):
    def test_motion_cases_round_trip_through_four_wheel_mixer(self):
        cases = ((1.0, 0.0), (0.0, 1.0), (0.6, -0.5), (-0.4, 0.2))
        for linear, angular in cases:
            with self.subTest(linear=linear, angular=angular):
                speeds = mix_twist(linear, angular, track_width=0.8,
                                   wheel_radius=0.2)
                recovered = wheels_to_twist(speeds.front_left, speeds.front_right,
                                            track_width=0.8, wheel_radius=0.2)
                self.assertAlmostEqual(recovered[0], linear)
                self.assertAlmostEqual(recovered[1], angular)

    def test_gate_stops_for_disabled_stale_and_emergency_conditions(self):
        common = dict(timeout=0.25, linear=0.3, angular=0.2,
                      max_linear=0.5, max_angular=1.0)
        for enabled, emergency_stop, command_age, localization_age in (
                (False, False, 0.0, 0.0),
                (True, True, 0.0, 0.0),
                (True, False, 0.251, 0.0),
                (True, False, 0.0, 0.251)):
            with self.subTest(enabled=enabled, emergency_stop=emergency_stop,
                              command_age=command_age,
                              localization_age=localization_age):
                self.assertEqual(
                    gated_velocity(enabled, emergency_stop, command_age,
                                   localization_age, **common),
                    (0.0, 0.0))

    def test_gate_saturates_only_when_both_inputs_are_fresh(self):
        output = gated_velocity(True, False, 0.01, 0.01, 0.25,
                                9.0, -9.0, 0.5, 1.0)
        self.assertEqual(output, (0.5, -1.0))

    def test_nonfinite_kinematic_and_gate_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            mix_twist(math.nan, 0.0, 0.8, 0.2)
        with self.assertRaises(ValueError):
            gated_velocity(True, False, 0.0, math.nan, 0.25,
                           0.0, 0.0, 0.5, 1.0)


if __name__ == '__main__':
    unittest.main()
