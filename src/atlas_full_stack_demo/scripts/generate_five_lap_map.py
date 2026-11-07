#!/usr/bin/env python3
"""Generate deterministic, separate Lanelet2/PCD assets for the synthetic loop demo."""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

LAT0 = 51.08080935188234
LON0 = -114.1331143659806
R_EARTH = 6378137.0
RADIUS_M = 5.0
STRAIGHT_M = 10.0
ROAD_WIDTH_M = 4.0
LANDMARK_SPACING_M = 4.0
LANDMARK_LATERAL_M = 7.5
# Keep the start lanelet outside Autoware's 3 m first-nearest trajectory
# association window from the terminal arc. A closer seam can make the
# smoother/controller reacquire the beginning of this open mission route.
LOOP_START_OFFSET_M = 8.0


def loop_segments():
    r, d = RADIUS_M, STRAIGHT_M
    nline = int(d) + 1
    nstart = max(2, int(math.ceil((d-LOOP_START_OFFSET_M)/0.4)) + 1)
    narc = 10
    return [
        [(x, -r) for x in [LOOP_START_OFFSET_M + (d-LOOP_START_OFFSET_M) * i / (nstart - 1)
                           for i in range(nstart)]],
        [(d + r * math.cos(a), r * math.sin(a)) for a in [-math.pi/2 + (math.pi/2)*i/narc for i in range(narc+1)]],
        [(d+r, y) for y in [d * i / (nline - 1) for i in range(nline)]],
        [(d + r * math.cos(a), d + r * math.sin(a)) for a in [(math.pi/2)*i/narc for i in range(narc+1)]],
        [(x, d+r) for x in [d - d * i / (nline - 1) for i in range(nline)]],
        [(r * math.cos(a), d + r * math.sin(a)) for a in [math.pi/2 + (math.pi/2)*i/narc for i in range(narc+1)]],
        [(-r, y) for y in [d - d * i / (nline - 1) for i in range(nline)]],
        [(r * math.cos(a), r * math.sin(a)) for a in [math.pi + (math.pi/2)*i/narc for i in range(narc+1)]],
    ]


def route_segments():
    # Keep the first segment separated from the nearby terminal arc. The
    # trajectory follower uses a first-nearest search with a 3 m soft distance
    # limit; a smaller offset aliases terminal poses to the route start.
    exit_radius = RADIUS_M
    exit_center = (0.0, -2.0*RADIUS_M)
    exit_arc = [(exit_center[0] + exit_radius*math.cos(a),
                 exit_center[1] + exit_radius*math.sin(a))
                for a in [math.pi/2-(math.pi/2)*i/12 for i in range(13)]]
    return loop_segments() + [[(0.0, -RADIUS_M)] + exit_arc[1:] +
                              [(exit_radius, -3.0*RADIUS_M)]]


def flatten_segments(parts):
    return [pt for i, part in enumerate(parts) for pt in (part if i == 0 else part[1:])]


def flatten_loop():
    return flatten_segments(loop_segments() + [loop_connector_segment()])


def loop_connector_segment():
    """Physical road connector from the terminal branch back to route start."""
    return [(0.0, -RADIUS_M), (LOOP_START_OFFSET_M, -RADIUS_M)]


def landmark_positions():
    """Place stable map features within range but outside the drivable corridor."""
    path = flatten_loop()
    full_route = flatten_route()
    lengths = [math.dist(a, b) for a, b in zip(path, path[1:])]
    total = sum(lengths)

    def clearance(point):
        best = math.inf
        for a, b in zip(full_route, full_route[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            t = max(0.0, min(1.0, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
            best = min(best, math.dist(point, (a[0]+t*dx, a[1]+t*dy)))
        return best

    positions = []
    for landmark_index, target in enumerate(
            [LANDMARK_SPACING_M/2 + i*LANDMARK_SPACING_M
             for i in range(int(math.ceil((total-LANDMARK_SPACING_M/2)/LANDMARK_SPACING_M)))]):
        remaining = target
        for i, length in enumerate(lengths):
            if remaining <= length:
                ratio = remaining/length
                a, b = path[i], path[i+1]
                x, y = a[0]+ratio*(b[0]-a[0]), a[1]+ratio*(b[1]-a[1])
                dx, dy = b[0]-a[0], b[1]-a[1]
                norm = math.hypot(dx, dy)
                candidates = []
                for lateral in [LANDMARK_LATERAL_M+i*0.5 for i in range(8)]:
                    for side in (-1.0, 1.0):
                        candidate = (x-side*dy/norm*lateral, y+side*dx/norm*lateral)
                        candidates.append((clearance(candidate), candidate))
                best_clearance, best_point = max(candidates, key=lambda item: item[0])
                if best_clearance < 5.5:
                    break
                positions.append(best_point)
                break
            remaining -= length
    return tuple(positions)


def flatten_route():
    return flatten_segments(route_segments())


LANDMARKS = landmark_positions()


def world_points():
    points = []
    # A flat, map-aligned synthetic surface plus separated non-planar landmarks.
    # This is a repeatable NDT fixture, not ray-traced or surveyed terrain.
    for path in (flatten_route(), loop_connector_segment()):
        for i, (x, y) in enumerate(path):
            if i and math.dist(path[i-1], (x, y)) < 0.5:
                continue
            if i+1 < len(path):
                dx, dy = path[i+1][0]-x, path[i+1][1]-y
            else:
                dx, dy = x-path[i-1][0], y-path[i-1][1]
            norm = math.hypot(dx, dy)
            nx, ny = -dy/norm, dx/norm
            for lateral in [j*0.5 for j in range(-16, 17)]:
                points.append((x+nx*lateral, y+ny*lateral, 0.0, 10.0))
    for cx, cy in LANDMARKS:
        # Surface samples on compact cuboids placed well outside the route
        # corridor, so LiDAR localization landmarks are not false obstacles.
        for ix in range(9):
            for iy in range(7):
                for iz in range(5):
                    points.append((cx-0.4+0.1*ix, cy-0.3+0.1*iy, 0.2+0.4*iz, 40.0))
    return points


def geodetic(x, y):
    # Small-area ENU inverse approximation. Error is sub-centimeter over this
    # 25 m test map; the map is synthetic and does not establish field geodesy.
    lat = LAT0 + math.degrees(y / R_EARTH)
    lon = LON0 + math.degrees(x / (R_EARTH * math.cos(math.radians(LAT0))))
    return lat, lon


def write_osm(path):
    root = ET.Element('osm', version='0.6', generator='atlas-five-lap-synthetic-map')
    ET.SubElement(root, 'MetaInfo', format_version='2', map_version='1', validation_version='1')
    next_node = 1
    nodes = {}

    def node_id(x, y):
        nonlocal next_node
        key = (round(x, 8), round(y, 8))
        if key not in nodes:
            lat, lon = geodetic(x, y)
            ET.SubElement(root, 'node', id=str(next_node), visible='true', version='1', lat=f'{lat:.12f}', lon=f'{lon:.12f}')
            nodes[key] = next_node
            next_node += 1
        return nodes[key]

    segments = route_segments()
    centerline, segment_indices = [], []
    for segment in segments:
        if not centerline:
            ids = list(range(len(segment)))
            centerline.extend(segment)
        else:
            ids = [len(centerline)-1]
            for point in segment[1:]:
                ids.append(len(centerline))
                centerline.append(point)
        segment_indices.append(ids)

    loop_end_index = len(flatten_segments(loop_segments())) - 1

    def normal_at(index):
        if index == 0:
            before, after = centerline[0], centerline[1]
        elif index == loop_end_index:
            before, after = centerline[index-1], centerline[index]
        elif index == len(centerline)-1:
            before, after = centerline[index-1], centerline[index]
        else:
            before, after = centerline[index-1], centerline[index+1]
        dx, dy = after[0]-before[0], after[1]-before[1]
        norm = math.hypot(dx, dy)
        return -dy/norm, dx/norm

    next_way = 1000
    for lane_index, indices in enumerate(segment_indices):
        left, right = [], []
        for global_index in indices:
            x, y = centerline[global_index]
            nx, ny = normal_at(global_index)
            half = ROAD_WIDTH_M/2
            left.append((x+nx*half, y+ny*half))
            right.append((x-nx*half, y-ny*half))
        left_id, right_id = next_way, next_way+1
        next_way += 2
        for way_id, coords in ((left_id, left), (right_id, right)):
            way = ET.SubElement(root, 'way', id=str(way_id), visible='true', version='1')
            for x, y in coords:
                ET.SubElement(way, 'nd', ref=str(node_id(x, y)))
            ET.SubElement(way, 'tag', k='type', v='line_thin')
            ET.SubElement(way, 'tag', k='subtype', v='solid')
        relation = ET.SubElement(root, 'relation', id=str(100+lane_index), visible='true', version='1')
        ET.SubElement(relation, 'member', type='way', ref=str(right_id), role='right')
        ET.SubElement(relation, 'member', type='way', ref=str(left_id), role='left')
        for key, value in (('subtype', 'road'), ('location', 'urban'), ('type', 'lanelet')):
            ET.SubElement(relation, 'tag', k=key, v=value)

    # A distinct lanelet closes the drivable loop from the terminal branch
    # back to the start lanelet. It is a branch in the map, not part of the
    # one-lap mission route to the terminal spur.
    start_normal = normal_at(0)
    finish_normal = normal_at(loop_end_index)
    connector_left = [(finish_normal[0]*ROAD_WIDTH_M/2,
                       -RADIUS_M+finish_normal[1]*ROAD_WIDTH_M/2),
                      (LOOP_START_OFFSET_M+start_normal[0]*ROAD_WIDTH_M/2,
                       -RADIUS_M+start_normal[1]*ROAD_WIDTH_M/2)]
    connector_right = [(-finish_normal[0]*ROAD_WIDTH_M/2,
                        -RADIUS_M-finish_normal[1]*ROAD_WIDTH_M/2),
                       (LOOP_START_OFFSET_M-start_normal[0]*ROAD_WIDTH_M/2,
                        -RADIUS_M-start_normal[1]*ROAD_WIDTH_M/2)]
    connector_way_ids = (next_way, next_way + 1)
    for way_id, coords in zip(connector_way_ids, (connector_right, connector_left)):
        way = ET.SubElement(root, 'way', id=str(way_id), visible='true', version='1')
        for x, y in coords:
            ET.SubElement(way, 'nd', ref=str(node_id(x, y)))
        ET.SubElement(way, 'tag', k='type', v='line_thin')
        ET.SubElement(way, 'tag', k='subtype', v='solid')
    relation = ET.SubElement(root, 'relation', id='109', visible='true', version='1')
    ET.SubElement(relation, 'member', type='way', ref=str(connector_way_ids[0]), role='right')
    ET.SubElement(relation, 'member', type='way', ref=str(connector_way_ids[1]), role='left')
    for key, value in (('subtype', 'road'), ('location', 'urban'), ('type', 'lanelet')):
        ET.SubElement(relation, 'tag', k=key, v=value)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def write_pcd(path):
    points = world_points()
    with path.open('w', encoding='ascii') as f:
        f.write('# Synthetic geometry aligned with five-lap Lanelet test map\n')
        f.write('VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
        f.write(f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n')
        for p in points:
            f.write('%.4f %.4f %.4f %.1f\n' % p)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parents[1] / 'config' / 'five_lap_loop')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_osm(args.output_dir / 'five_lap_loop.osm')
    write_pcd(args.output_dir / 'five_lap_loop.pcd')
    (args.output_dir / 'map_projector_info.yaml').write_text(
        'projector_type: LocalCartesian\nvertical_datum: WGS84\nmap_origin:\n'
        f'  latitude: {LAT0}\n  longitude: {LON0}\n  altitude: 0.0\n', encoding='utf-8')
    print(f'generated deterministic synthetic loop map in {args.output_dir}')


if __name__ == '__main__':
    main()
