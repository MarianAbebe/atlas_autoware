#!/usr/bin/env python3
import math
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_five_lap_map import (LANDMARKS, LOOP_START_OFFSET_M, flatten_loop,
    flatten_route, flatten_segments, loop_segments, route_segments, write_osm, write_pcd)


class FiveLapMapGeometry(unittest.TestCase):
    @staticmethod
    def distance_to_route(point, route):
        best = math.inf
        for a, b in zip(route, route[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            t = max(0.0, min(1.0, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
            best = min(best, math.dist(point, (a[0]+t*dx, a[1]+t*dy)))
        return best

    def test_map_has_connected_four_turn_loop_and_distinct_mission_seam(self):
        segments = route_segments()
        self.assertEqual(len(segments), 9)
        for left, right in zip(segments, segments[1:]):
            self.assertLess(math.dist(left[-1], right[0]), 1e-9)
        loop = flatten_loop()
        self.assertLess(math.dist(loop[0], loop[-1]), 1e-9)
        length = sum(math.dist(a, b) for a, b in zip(loop, loop[1:]))
        self.assertGreater(length, 50.0)
        self.assertLess(length, 100.0)
        headings = [math.atan2(b[1]-a[1], b[0]-a[0]) for a, b in zip(loop, loop[1:])]
        turns = [abs(math.atan2(math.sin(b-a), math.cos(b-a))) for a, b in zip(headings, headings[1:])]
        active = [delta > 0.03 for delta in turns]
        groups = sum(flag and (i == 0 or not active[i-1]) for i, flag in enumerate(active))
        curvature = [delta / math.dist(loop[i], loop[i+1]) for i, delta in enumerate(turns)]
        self.assertEqual(groups, 4)
        # 5 m radius, allowing discretization error from 10 chord segments per corner.
        self.assertLess(max(curvature), 0.22)
        route = flatten_route()
        self.assertGreater(math.dist(route[0], route[-1]), 2.0)
        self.assertGreater(math.dist(route[-1], loop[0]), 2.0)
        mission_loop = flatten_segments(loop_segments())
        self.assertAlmostEqual(mission_loop[0][0], LOOP_START_OFFSET_M)
        self.assertAlmostEqual(mission_loop[0][1], -5.0)
        self.assertAlmostEqual(mission_loop[-1][0], 0.0)
        self.assertAlmostEqual(math.dist(mission_loop[0], mission_loop[-1]), LOOP_START_OFFSET_M)
        self.assertGreater(sum(math.dist(a, b) for a, b in zip(mission_loop, mission_loop[1:])), 63.0)
        self.assertGreater(self.distance_to_route(mission_loop[0], segments[-1]), 2.0)
        # The terminal arc must not be soft-associated with the first route
        # segment by Autoware's configured 3 m nearest-point distance.
        terminal_to_start = min(self.distance_to_route(point, segments[0])
                                for point in segments[-1])
        self.assertGreater(terminal_to_start, 3.2)
        self.assertGreaterEqual(min(self.distance_to_route(p, route) for p in LANDMARKS), 5.5)
        cumulative = [0.0]
        for a, b in zip(loop, loop[1:]):
            cumulative.append(cumulative[-1] + math.dist(a, b))
        for sample in range(0, int(length)+1):
            index = next(i for i, s in enumerate(cumulative[1:]) if s >= sample)
            a, b = loop[index], loop[index+1]
            ratio = (sample-cumulative[index])/math.dist(a, b)
            position = (a[0]+ratio*(b[0]-a[0]), a[1]+ratio*(b[1]-a[1]))
            self.assertLessEqual(min(math.dist(position, landmark) for landmark in LANDMARKS), 11.8)

    def test_lanelet_and_pointcloud_assets_are_generated_separately(self):
        with tempfile.TemporaryDirectory() as directory:
            osm, pcd = Path(directory) / 'loop.osm', Path(directory) / 'loop.pcd'
            write_osm(osm)
            write_pcd(pcd)
            text = osm.read_text()
            self.assertEqual(text.count('k="type" v="lanelet"'), 10)
            self.assertEqual(text.count('k="subtype" v="road"'), 10)
            self.assertEqual(text.count('k="type" v="line_thin"'), 20)
            root = ET.parse(osm).getroot()
            meta = root.find('MetaInfo')
            self.assertIsNotNone(meta)
            self.assertEqual(meta.get('format_version'), '2')
            ways = {w.get('id'): [n.get('ref') for n in w.findall('nd')] for w in root.findall('way')}
            lanelets = []
            for relation in root.findall('relation'):
                members = {m.get('role'): ways[m.get('ref')] for m in relation.findall('member')}
                lanelets.append(members)
            for current, following in zip(lanelets[:7], lanelets[1:8]):
                self.assertEqual(current['left'][-1], following['left'][0])
                self.assertEqual(current['right'][-1], following['right'][0])
            self.assertEqual(lanelets[7]['left'][-1], lanelets[8]['left'][0])
            self.assertEqual(lanelets[7]['right'][-1], lanelets[8]['right'][0])
            self.assertEqual(lanelets[7]['left'][-1], lanelets[9]['left'][0])
            self.assertEqual(lanelets[7]['right'][-1], lanelets[9]['right'][0])
            self.assertEqual(lanelets[9]['left'][-1], lanelets[0]['left'][0])
            self.assertEqual(lanelets[9]['right'][-1], lanelets[0]['right'][0])
            header = pcd.read_text().split('DATA ascii\n', 1)[0]
            self.assertIn('FIELDS x y z intensity', header)
            self.assertIn('WIDTH ', header)
            self.assertGreater(pcd.stat().st_size, 1000)


if __name__ == '__main__':
    unittest.main()
