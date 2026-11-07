import math
import unittest
from atlas_nav2_demo.ackermann_kinematics import AckermannState, step, turning_radius


class AckermannKinematicsTest(unittest.TestCase):
    def test_straight_reverse_and_curved_motion(self):
        state, yaw_rate = step(AckermannState(), 1., 0., 1., 1.2, .5, 1., 2., 3., 2.)
        self.assertGreater(state.x, 0.0); self.assertEqual(yaw_rate, 0.0)
        reverse, _ = step(state, -1., 0., 1., 1.2, .5, 1., 3., 3., 2.)
        self.assertLess(reverse.speed, 0.0)
        curved, yaw_rate = step(AckermannState(speed=1.), 1., .4, 1., 1.2, .5, 2., 3., 3., 2.)
        self.assertGreater(yaw_rate, 0.0); self.assertGreater(curved.yaw, 0.0)

    def test_limits_and_minimum_radius(self):
        state, _ = step(AckermannState(), 9., 9., 1., 1.2, .5, .2, 1., 1., .8)
        self.assertLessEqual(abs(state.speed), .8); self.assertLessEqual(abs(state.steering), .2)
        self.assertAlmostEqual(turning_radius(1.2, .5), 1.2 / math.tan(.5))

    def test_zero_speed_cannot_rotate(self):
        state, yaw_rate = step(AckermannState(), 0., .4, 1., 1.2, .5, 1., 1., 1., 1.)
        self.assertEqual(yaw_rate, 0.0); self.assertEqual(state.yaw, 0.0)


if __name__ == '__main__': unittest.main()
