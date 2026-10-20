#!/usr/bin/env python3
"""Recompute tracking/stop evidence from raw saved recordings without ROS."""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def project(rows, poly):
    poly = poly[np.r_[True, np.linalg.norm(np.diff(poly, axis=0), axis=1) > 1e-8]]
    seg = np.diff(poly, axis=0)
    lat, heading = [], []
    for row in rows:
        rel = row[:2] - poly[:-1]
        u = np.clip(np.sum(rel * seg, axis=1) / np.sum(seg * seg, axis=1), 0, 1)
        nearest = np.argmin(np.linalg.norm(poly[:-1] + u[:, None] * seg - row[:2], axis=1))
        d, r = seg[nearest], rel[nearest]
        lat.append(abs(d[0]*r[1] - d[1]*r[0]) / np.linalg.norm(d))
        angle = row[2] - math.atan2(d[1], d[0])
        heading.append(abs(math.atan2(math.sin(angle), math.cos(angle))))
    return {'max_lateral_error_m': float(max(lat)),
            'rms_lateral_error_m': float(np.sqrt(np.mean(np.square(lat)))),
            'max_heading_error_rad': float(max(heading))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('recording', type=Path)
    args = parser.parse_args()
    path = args.recording
    state = np.genfromtxt(path/'odometry.csv', delimiter=',', names=True)
    route = np.genfromtxt(path/'route.csv', delimiter=',', names=True)
    trajectory = np.genfromtxt(path/'initial_trajectory.csv', delimiter=',', names=True)
    report = json.loads((path/'validation.json').read_text())
    xy = np.column_stack([state['x'], state['y']])
    tail = state[state['wall_sec'] >= state['wall_sec'][-1] - 5]
    dt = np.diff(state['stamp'])
    acceleration = np.diff(state['speed'])[dt > 1e-6]/dt[dt > 1e-6]
    out = {'recording': str(path), 'historical_report_passed': report['passed'],
           'total_recorded_distance_m': float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
           'initial_recorded_start_gap_m': math.hypot(state['x'][0]-route['x'][0],state['y'][0]-route['y'][0]),
           'final_goal_distance_m': math.hypot(state['x'][-1]-route['x'][-1],state['y'][-1]-route['y'][-1]),
           'max_actual_speed_mps': float(max(state['speed'])),
           'max_acceleration_mps2': float(max(acceleration)),
           'max_deceleration_mps2': float(min(acceleration)),
           'final_speed_mps': float(state['speed'][-1]),
           'final_5s_max_abs_speed_mps': float(max(abs(tail['speed']))),
           'final_5s_distance_m': float(np.linalg.norm(np.diff(np.column_stack([tail['x'],tail['y']]),axis=0),axis=1).sum()),
           'strict_standstill_passed': bool(max(abs(tail['speed'])) < .001),
           'reference': 'initial pinned smoothed trajectory; perpendicular segment distance',
           **project(np.column_stack([state['x'],state['y'],state['yaw']]),np.column_stack([trajectory['x'],trajectory['y']]))}
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
