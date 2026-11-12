import math
import unittest

from atlas_nav2_demo.ackermann_adapter_logic import convert_command


class AckermannAdapterLogicTest(unittest.TestCase):
    def test_feasible_curvature_converts_to_steering(self):
        velocity, curvature, steering, feasible = convert_command(
            0.5, 0.1, 1.2, 0.5, 0.8, 0.05)
        self.assertTrue(feasible)
        self.assertAlmostEqual(velocity, 0.5)
        self.assertAlmostEqual(curvature, 0.2)
        self.assertAlmostEqual(steering, math.atan(0.24))

    def test_infeasible_curvature_is_rejected_without_clipping(self):
        velocity, curvature, steering, feasible = convert_command(
            0.2, 1.0, 1.2, 0.5, 0.8, 0.05)
        self.assertFalse(feasible)
        self.assertEqual(velocity, 0.0)
        self.assertAlmostEqual(curvature, 5.0)
        self.assertGreater(steering, 0.5)

    def test_near_zero_speed_rotation_is_rejected(self):
        velocity, curvature, steering, feasible = convert_command(
            0.0, 0.4, 1.2, 0.5, 0.8, 0.05)
        self.assertFalse(feasible)
        self.assertEqual((velocity, curvature, steering), (0.0, 0.0, 0.0))

    def test_speed_is_bounded(self):
        velocity, _, _, feasible = convert_command(
            2.0, 0.0, 1.2, 0.5, 0.8, 0.05)
        self.assertTrue(feasible)
        self.assertEqual(velocity, 0.8)

    def test_nonfinite_commands_are_rejected(self):
        velocity, curvature, steering, feasible = convert_command(
            float('nan'), 0.0, 1.2, 0.5, 0.8, 0.05)
        self.assertFalse(feasible)
        self.assertEqual((velocity, curvature, steering), (0.0, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
