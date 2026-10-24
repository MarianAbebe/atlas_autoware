#!/usr/bin/env python3
"""Bounded ROS/DDS integration validation; deliberately performs no screenshots."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from autoware_map_msgs.msg import LaneletMapBin, MapProjectorInfo
from autoware_planning_msgs.msg import LaneletRoute
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--hold-sec', type=float, default=8.0)
    args = parser.parse_args()
    result_dir = Path('/tmp/atlas_map_demo_results') / time.strftime('run_%Y%m%d_%H%M%S')
    result_dir.mkdir(parents=True)
    map_dir = Path(os.environ.get('ATLAS_MAP_DIR', Path.home() / 'Downloads')).expanduser()
    originals = [map_dir / f for f in
                 ['finalMap_repaired.osm', 'pointcloud_map_georeferenced.pcd', 'map_projector_info.yaml']]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in originals}
    rclpy.init()
    node = rclpy.create_node('atlas_map_runtime_validation')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    samples, first_received = {}, {}
    begin = time.monotonic()
    def receive(name, msg):
        if name not in samples:
            samples[name] = msg
            first_received[name] = time.monotonic() - begin
    topics = {
        'projector': ('/map/map_projector_info', MapProjectorInfo),
        'map': ('/atlas_map_demo/vector_map', LaneletMapBin),
        'pcd': ('/atlas_map_demo/pointcloud_map', PointCloud2),
        'roads': ('/atlas_map_demo/road_markers', MarkerArray),
        'route': ('/atlas_map_demo/route', LaneletRoute),
        'path': ('/atlas_map_demo/route_path', RosPath),
        'diagnostics': ('/atlas_map_demo/diagnostic_markers', MarkerArray),
    }
    subscriptions = [node.create_subscription(kind, topic,
                      lambda msg, name=name: receive(name, msg), qos)
                     for name, (topic, kind) in topics.items()]
    env = dict(os.environ, ROS_LOG_DIR=str(result_dir / 'ros_logs'), QT_QPA_PLATFORM='xcb')
    log = result_dir / 'runtime.log'
    checks = {}
    observed_nodes = set()
    rviz_subscriptions = set()
    last_live_check = None
    with log.open('w') as output:
        process = subprocess.Popen(
            ['ros2', 'launch', 'atlas_map_integration_demo', 'campus_map.launch.py',
             f'results_dir:={result_dir}', f'launch_rviz:={str(not args.no_rviz).lower()}'],
            stdout=output, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        ready_time = None
        try:
            while time.monotonic() - begin < 75 and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
                if (result_dir / 'failure.txt').exists():
                    break
                if len(samples) != len(topics) or not (result_dir / 'routing.json').exists():
                    continue
                if ready_time is None:
                    ready_time = time.monotonic()
                nodes = {name for name, ns in node.get_node_names_and_namespaces() if ns == '/atlas_map_demo'}
                observed_nodes |= nodes
                expected = {'map_container', 'map_projection_loader', 'lanelet2_map_loader',
                            'pointcloud_map_loader', 'lanelet2_map_visualization', 'map_routing_probe'}
                if not args.no_rviz:
                    expected.add('campus_rviz')
                    for name in ['pcd', 'roads', 'path', 'diagnostics']:
                        endpoints = node.get_subscriptions_info_by_topic(topics[name][0])
                        if any(e.node_name == 'campus_rviz' for e in endpoints):
                            rviz_subscriptions.add(name)
                last_live_check = expected.issubset(nodes)
                if time.monotonic() - ready_time >= args.hold_sec:
                    break
            checks['nodes_alive_during_hold'] = last_live_check is True and process.poll() is None
            checks['all_topics_received'] = len(samples) == len(topics)
            checks['rviz_subscribed_all_displays'] = args.no_rviz or rviz_subscriptions == {'pcd', 'roads', 'path', 'diagnostics'}
        finally:
            if process.poll() is None:
                os.kill(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
    records = {}
    for name, msg in samples.items():
        record = {'topic': topics[name][0], 'message_type': msg.__class__.__module__ + '.' + msg.__class__.__name__,
                  'received_sec_since_launch': first_received[name]}
        if hasattr(msg, 'header'):
            record.update(frame=msg.header.frame_id, stamp_sec=msg.header.stamp.sec,
                          stamp_nanosec=msg.header.stamp.nanosec)
        if name == 'map':
            record['payload_bytes'] = len(msg.data)
            checks['map_frame_nonempty'] = msg.header.frame_id == 'map' and len(msg.data) > 0
        elif name == 'pcd':
            record.update(points=msg.width * msg.height, payload_bytes=len(msg.data), point_step=msg.point_step)
            fields = {f.name: f for f in msg.fields}
            checks['pcd_frame_xyz_fields'] = msg.header.frame_id == 'map' and all(k in fields and fields[k].datatype == 7 for k in ['x', 'y', 'z'])
            dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                              'offsets': [fields[k].offset for k in ['x', 'y', 'z']], 'itemsize': msg.point_step})
            published = np.frombuffer(msg.data, dtype=dtype)
            with originals[1].open('rb') as f:
                while not f.readline().startswith(b'DATA '):
                    pass
                original = np.fromfile(f, dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('padding', '<f4')]))
            same = len(published) == len(original) and all(np.array_equal(published[k], original[k]) for k in ['x', 'y', 'z'])
            checks['original_pcd_xyz_exactly_preserved'] = same
            record['xyz_exactly_equal_original'] = same
        elif name in ['roads', 'diagnostics']:
            record['markers'] = len(msg.markers)
            checks[name + '_markers_valid'] = bool(msg.markers) and all(m.header.frame_id == 'map' for m in msg.markers)
        elif name == 'path':
            record['poses'] = len(msg.poses)
            checks['route_path_valid'] = msg.header.frame_id == 'map' and len(msg.poses) > 2 and all(p.header.frame_id == 'map' for p in msg.poses)
        elif name == 'route':
            record['segments'] = len(msg.segments)
            graph = json.loads((result_dir / 'routing.json').read_text())
            checks['route_matches_computed_path'] = msg.header.frame_id == 'map' and [s.preferred_primitive.id for s in msg.segments] == graph['route_lanelet_ids']
        elif name == 'projector':
            record.update(projector=msg.projector_type, vertical_datum=msg.vertical_datum,
                          origin=[msg.map_origin.latitude, msg.map_origin.longitude, msg.map_origin.altitude])
            checks['projector_localcartesian'] = msg.projector_type == 'LocalCartesian' and msg.vertical_datum == 'WGS84'
        records[name] = record
    text = log.read_text()
    checks['no_critical_runtime_errors'] = '[ERROR]' not in text and 'FAIL:' not in text and 'process has died' not in text
    checks['probe_pass_logged'] = 'PASS: original map publications' in text
    checks['clean_launch_exit'] = process.returncode == 0
    checks['original_hashes_unchanged'] = all(hashlib.sha256(p.read_bytes()).hexdigest() == hashes[str(p)] for p in originals)
    result = {'passed': all(checks.values()), 'checks': checks, 'topics': records,
              'nodes': sorted(observed_nodes), 'rviz_subscriptions': sorted(rviz_subscriptions),
              'runtime_sec': time.monotonic() - begin, 'launch_exit': process.returncode,
              'original_sha256': hashes, 'results_dir': str(result_dir),
              'visual_inspection': 'Not performed; no screenshot capture attempted.'}
    (result_dir / 'runtime_validation.json').write_text(json.dumps(result, indent=2) + '\n')
    Path('/tmp/atlas_map_demo_results/latest_run.txt').write_text(str(result_dir) + '\n')
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
