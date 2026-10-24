#!/usr/bin/env python3
"""Read-only map alignment diagnostics; no estimated transform is applied."""
import argparse
import json
import os
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('results_dir', type=Path)
    args = parser.parse_args()
    map_dir = Path(os.environ.get('ATLAS_MAP_DIR', Path.home() / 'Downloads')).expanduser()
    with (map_dir / 'pointcloud_map_georeferenced.pcd').open('rb') as f:
        while not f.readline().startswith(b'DATA '):
            pass
        xyz = np.fromfile(f, dtype='<f4').reshape(-1, 4)[:, :3]
    bounds = np.genfromtxt(args.results_dir / 'boundaries.csv', delimiter=',', names=True)
    centers = np.genfromtxt(args.results_dir / 'centerlines.csv', delimiter=',', names=True)
    xy_min, xy_max = xyz[:, :2].min(axis=0), xyz[:, :2].max(axis=0)
    local = centers[(centers['x'] >= xy_min[0]) & (centers['x'] <= xy_max[0]) &
                    (centers['y'] >= xy_min[1]) & (centers['y'] <= xy_max[1])]
    # Minimum observed elevation in 1 m cells is a lower envelope, NOT an
    # automatically accepted road surface. Outliers/vegetation can bias it.
    cells = np.floor(xyz[:, :2]).astype(np.int32)
    unique, inv, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    low = np.full(len(unique), np.inf)
    np.minimum.at(low, inv, xyz[:, 2])
    accepted = counts >= 10
    cell_centers = unique[accepted] + 0.5
    lower = low[accepted]
    tree = cKDTree(cell_centers)
    d, ix = tree.query(np.column_stack([local['x'], local['y']]))
    nearby = d <= 1.5
    dz = lower[ix[nearby]] - local['z'][nearby]
    quantiles = lambda x: {str(q): float(np.quantile(x, q)) for q in [0, .1, .5, .9, 1]} if len(x) else {}
    result = {
        'pcd_points': len(xyz), 'pcd_min': xyz.min(axis=0).tolist(), 'pcd_max': xyz.max(axis=0).tolist(),
        'boundary_min': [float(bounds[k].min()) for k in ['x', 'y', 'z']],
        'boundary_max': [float(bounds[k].max()) for k in ['x', 'y', 'z']],
        'centerline_samples_in_pcd_bbox': len(local), 'local_lanelets': sorted(set(local['lanelet_id'].astype(int).tolist())),
        'sampled_centerlines_near_occupied_cells': int(nearby.sum()),
        'lower_envelope_minus_lanelet_z_m': quantiles(dz),
        'nearest_occupied_cell_xy_distance_m': quantiles(d),
        'method': 'Untransformed original PCD; minimum Z in 1m XY cells with at least 10 points; match runtime centerline samples within 1.5m.',
        'limitations': 'Occupancy proximity is NOT an XY registration residual. The lower envelope is NOT certified road ground. No structural correspondences or rigid transform are assumed.',
        'transform_applied': False,
    }
    (args.results_dir / 'alignment.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
