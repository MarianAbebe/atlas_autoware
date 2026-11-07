#!/usr/bin/env python3
import unittest

from lap_metrics import (extract_closed_prefix, extract_loop_prefix, prepare_route, project_pose,
                         nearest_trajectory_reference, terminal_goal_stopped, unwrap_progress)


class ClosedRouteMetrics(unittest.TestCase):
    def test_projection_and_lap_seam_unwrap(self):
        route, lengths, cumulative, total = prepare_route(
            [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)])
        cross, s, yaw = project_pose(5, 0.5, route, lengths, cumulative)
        self.assertAlmostEqual(cross, 0.5)
        self.assertAlmostEqual(s, 5.0)
        self.assertAlmostEqual(yaw, 0.0)
        self.assertAlmostEqual(unwrap_progress(total-0.1, 0.1, total), 0.2)

    def test_invalid_open_or_degenerate_route_rejected(self):
        with self.assertRaises(ValueError):
            prepare_route([(0, 0), (1, 0), (2, 0)])
        with self.assertRaises(ValueError):
            prepare_route([(0, 0), (1, 0), (1, 0), (0, 0)])

    def test_open_mission_extracts_closed_lap_prefix(self):
        prefix = extract_closed_prefix(
            [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0), (2, 0)], 40.0)
        self.assertEqual(prefix[-1], (0.0, 0.0))
        _, _, _, length = prepare_route(prefix)
        self.assertEqual(length, 40.0)

    def test_open_mission_rejects_bad_loop_length_or_open_prefix(self):
        for length in (0.0, 20.0, 50.0):
            with self.subTest(length=length), self.assertRaises(ValueError):
                extract_closed_prefix([(0, 0), (10, 0), (10, 10), (0, 10), (0, 0), (2, 0)], length)
        with self.assertRaises(ValueError):
            extract_closed_prefix([(0, 0), (10, 0), (10, 10), (0, 10), (2, 0)], 40.0)

    def test_distinct_mission_seam_prefix_allows_bounded_open_endpoint(self):
        mission = [(1, 0), (10, 0), (10, 10), (0, 10), (0, 0), (2, 0)]
        prefix = extract_loop_prefix(mission, 39.5, maximum_seam_gap_m=1.5)
        self.assertEqual(prefix[0], (1.0, 0.0))
        self.assertEqual(prefix[-1], (0.5, 0.0))
        route, lengths, cumulative, total = prepare_route(prefix, require_closed=False)
        self.assertEqual(total, 39.5)
        self.assertAlmostEqual(unwrap_progress(total-0.1, 0.1, total), 0.2)
        self.assertAlmostEqual(project_pose(0.4, 0.0, route, lengths, cumulative)[1], total-0.1)

    def test_distinct_mission_seam_rejects_unbounded_gap(self):
        mission = [(4, 0), (10, 0), (10, 10), (0, 10), (0, 0), (2, 0)]
        with self.assertRaises(ValueError):
            extract_loop_prefix(mission, 37.0, maximum_seam_gap_m=1.5)

    def test_terminal_spur_goal_is_distinct_from_lap_seam(self):
        mission = [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0), (2, 0)]
        prefix = extract_closed_prefix(mission, 40.0)
        self.assertEqual(prefix[0], (0.0, 0.0))
        self.assertGreater(__import__('math').dist(mission[-1], prefix[0]), 1.0)

    def test_nearest_controller_reference_records_heading_velocity_and_ahead_points(self):
        from types import SimpleNamespace as Obj
        def point(x, y, yaw, velocity):
            return Obj(pose=Obj(position=Obj(x=x, y=y), orientation=Obj(
                x=0.0, y=0.0, z=__import__('math').sin(yaw/2), w=__import__('math').cos(yaw/2))),
                longitudinal_velocity_mps=velocity)
        points = [point(0, 0, 0, 1), point(1, 0, 0, 0.5),
                  point(1, -1, -__import__('math').pi/2, 0)]
        ref = nearest_trajectory_reference(points, (0.9, -0.1), ahead_count=1)
        self.assertEqual(ref['nearest_index'], 1)
        self.assertAlmostEqual(ref['distance_m'], 0.1*2**0.5)
        self.assertEqual(ref['points'][0]['velocity_mps'], 0.5)
        self.assertAlmostEqual(ref['points'][1]['yaw_rad'], -__import__('math').pi/2)
        self.assertIsNone(nearest_trajectory_reference([], (0, 0)))
        self.assertIsNone(nearest_trajectory_reference(points, (float('nan'), 0)))

    def test_terminal_hold_requires_spur_goal_and_stop_speed(self):
        goal = (2.0, 0.0, 0.0)
        self.assertFalse(terminal_goal_stopped((0.0, 0.0), 0.0, goal))
        self.assertFalse(terminal_goal_stopped((2.0, 0.0), 0.2, goal))
        self.assertTrue(terminal_goal_stopped((2.1, 0.0), 0.05, goal))


if __name__ == '__main__':
    unittest.main()
