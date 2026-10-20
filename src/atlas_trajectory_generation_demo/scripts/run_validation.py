#!/usr/bin/env python3
"""Bounded validation of actual pinned trajectory publications, never a generator."""
import argparse
import csv
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
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from autoware_internal_planning_msgs.msg import PathWithLaneId
from nav_msgs.msg import Path as RosPath, Odometry
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2


def geometry(points):
    xyz = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in points])
    q = np.array([[p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w] for p in points])
    ds = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)
    a, b, c = xyz[1:-1, :2] - xyz[:-2, :2], xyz[2:, :2] - xyz[1:-1, :2], xyz[2:, :2] - xyz[:-2, :2]
    den = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(c, axis=1)
    curvature = 2 * np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]) / np.maximum(den, 1e-15)
    return xyz, q, ds, curvature


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--route-only', action='store_true')
    parser.add_argument('--no-rviz', action='store_true')
    args = parser.parse_args()
    root = Path('/tmp/atlas_trajectory_results')
    run = root / (time.strftime('run_%Y%m%d_%H%M%S') + ('_route_only' if args.route_only else ''))
    run.mkdir(parents=True)
    originals = [Path('/home/marian/Downloads') / f for f in
                 ['finalMap_repaired.osm', 'map_projector_info.yaml', 'pointcloud_map_georeferenced.pcd']]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in originals}
    rclpy.init()
    node = rclpy.create_node('trajectory_validation')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    samples, received = {}, {}
    final_samples, path_stamps, raw_stamps = [], set(), set()
    begin = time.monotonic()
    def record(name, msg):
        samples[name] = msg
        received.setdefault(name, time.monotonic() - begin)
        if name == 'path':
            path_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
        if name == 'raw':
            raw_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
        if name == 'trajectory' and len(final_samples) < 20:
            final_samples.append(msg)
    topics = {
        'route': ('/atlas_trajectory_demo/route', LaneletRoute, latched),
        'route_path': ('/atlas_trajectory_demo/route_path', RosPath, latched),
        'ego': ('/atlas_trajectory_demo/synthetic/odometry', Odometry, 10),
        'markers': ('/atlas_trajectory_demo/markers', MarkerArray, 10),
        'pcd': ('/atlas_map_demo/pointcloud_map', PointCloud2, latched),
        'roads': ('/atlas_map_demo/road_markers', MarkerArray, latched),
    }
    if not args.route_only:
        topics.update({
            'path': ('/atlas_trajectory_demo/path_with_lane_id', PathWithLaneId, 10),
            'raw': ('/atlas_trajectory_demo/raw_trajectory', Trajectory, 10),
            'trajectory': ('/atlas_trajectory_demo/trajectory', Trajectory, 10),
            'trajectory_visualization': ('/atlas_trajectory_demo/trajectory_visualization', RosPath, 10)})
    subscriptions = [node.create_subscription(kind, topic, lambda msg, n=name: record(n, msg), qos)
                     for name, (topic, kind, qos) in topics.items()]
    checks, metrics, subscribers, observed_nodes = {}, {}, set(), set()
    ready = None
    log = run / 'runtime.log'
    env = dict(os.environ, ROS_LOG_DIR=str(run / 'ros_logs'), QT_QPA_PLATFORM='xcb')
    with log.open('w') as output:
        process = subprocess.Popen(
            ['ros2', 'launch', 'atlas_trajectory_generation_demo', 'trajectory_demo.launch.py',
             f'results_dir:={run}', f'planning_enabled:={str(not args.route_only).lower()}',
             f'launch_rviz:={str(not args.no_rviz).lower()}'],
            stdout=output, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        try:
            while time.monotonic() - begin < 75 and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
                if (run / 'failure.txt').exists():
                    break
                if len(samples) != len(topics) or (not args.route_only and len(final_samples) < 10):
                    continue
                if ready is None:
                    ready = time.monotonic()
                nodes = {n for n, ns in node.get_node_names_and_namespaces() if ns in ['/atlas_map_demo', '/atlas_trajectory_demo']}
                observed_nodes |= nodes
                expected = {'trajectory_fixture', 'lanelet2_map_loader', 'pointcloud_map_loader', 'lanelet2_map_visualization'}
                if not args.route_only:
                    expected |= {'path_generator', 'path_to_trajectory_converter', 'velocity_smoother'}
                if not args.no_rviz:
                    expected.add('trajectory_rviz')
                    for name in ['pcd', 'roads', 'route_path', 'markers'] + ([] if args.route_only else ['trajectory_visualization']):
                        if any(s.node_name == 'trajectory_rviz' for s in node.get_subscriptions_info_by_topic(topics[name][0])):
                            subscribers.add(name)
                checks['required_nodes_alive'] = expected.issubset(nodes)
                if time.monotonic() - ready >= 4:
                    break
        finally:
            if process.poll() is None:
                os.kill(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
    checks['all_required_topics_received'] = len(samples) == len(topics)
    checks['rviz_displays_subscribed'] = args.no_rviz or subscribers == set(['pcd', 'roads', 'route_path', 'markers'] + ([] if args.route_only else ['trajectory_visualization']))
    if 'route' in samples:
        route = samples['route']
        checks['chosen_route_actual_ids'] = [s.preferred_primitive.id for s in route.segments] == [38184, 23892]
        checks['route_map_frame'] = route.header.frame_id == 'map'
    if 'ego' in samples:
        ego = samples['ego']
        checks['synthetic_ego_frame_velocity'] = ego.header.frame_id == 'map' and ego.child_frame_id == 'atlas_trajectory_ego' and abs(ego.twist.twist.linear.x - .2) < 1e-8
    if 'pcd' in samples:
        checks['original_pcd_point_count_frame'] = samples['pcd'].width * samples['pcd'].height == 1004344 and samples['pcd'].header.frame_id == 'map'
    route_file = run / 'selected_route.json'
    selected = json.loads(route_file.read_text()) if route_file.exists() else {}
    checks['route_selected_verified'] = bool(selected) and selected.get('transition_gap_m', 1) < 1e-5 and selected.get('pcd_xy_bbox_coverage_fraction') == 1.0
    if 'route_path' in samples and len(samples['route_path'].poses) > 2:
        route_xyz, route_q, route_ds, route_curvature = geometry(samples['route_path'].poses)
        metrics.update(route_path_points=len(route_xyz), route_path_length_m=float(route_ds.sum()),
                       route_max_sampled_curvature_per_m=float(route_curvature.max()))
        checks['route_path_finite_continuous'] = bool(np.isfinite(route_xyz).all() and np.isfinite(route_q).all() and route_ds.min() > 1e-5 and route_ds.max() < 1.5)
        checks['route_path_length_consistent'] = bool(abs(route_ds.sum() - selected.get('route_length_m', 0)) < 1e-4)
        checks['route_path_inside_pcd_xy_bbox'] = bool(np.all(route_xyz[:, 0] >= -79.563538) and np.all(route_xyz[:, 0] <= 122.839691) and np.all(route_xyz[:, 1] >= -76.59005) and np.all(route_xyz[:, 1] <= 119.276711))
    if not args.route_only:
        checks['actual_smoother_samples'] = len(final_samples) >= 10
        if final_samples and 'route' in samples:
            sample_metrics = []
            for output in final_samples:
                if len(output.points) < 3:
                    continue
                sample_xyz, sample_q, sample_ds, sample_k = geometry(output.points)
                sample_v = np.array([p.longitudinal_velocity_mps for p in output.points])
                sample_a = np.array([p.acceleration_mps2 for p in output.points])
                sample_spatial_a = np.diff(sample_v ** 2) / (2 * np.maximum(sample_ds, 1e-15))
                sample_metrics.append({
                    'points': len(output.points), 'length_m': float(sample_ds.sum()),
                    'max_curvature_per_m': float(sample_k.max()),
                    'min_speed_mps': float(sample_v.min()), 'max_speed_mps': float(sample_v.max()),
                    'max_acceleration_mps2': float(sample_a.max()), 'min_acceleration_mps2': float(sample_a.min()),
                    'spatial_max_acceleration_mps2': float(sample_spatial_a.max()),
                    'spatial_min_acceleration_mps2': float(sample_spatial_a.min()),
                    'terminal_speed_mps': float(sample_v[-1]),
                    'finite': bool(np.isfinite(sample_xyz).all() and np.isfinite(sample_q).all() and
                                   np.isfinite(sample_v).all() and np.isfinite(sample_a).all()),
                    'valid_geometry': bool(sample_ds.min() > 1e-5 and sample_ds.max() < 1.5 and
                                           sample_k.max() <= np.tan(.45) / 1.2 and
                                           np.max(np.abs(np.linalg.norm(sample_q, axis=1) - 1)) < 1e-4),
                })
            checks['all_sampled_outputs_finite_valid_geometry'] = len(sample_metrics) == len(final_samples) and all(
                m['finite'] and m['valid_geometry'] for m in sample_metrics)
            checks['all_sampled_outputs_velocity_acceleration_stop_bounds'] = all(
                m['min_speed_mps'] >= -1e-6 and m['max_speed_mps'] <= 1.0001 and
                m['max_acceleration_mps2'] <= .53 and m['min_acceleration_mps2'] >= -.53 and
                m['spatial_max_acceleration_mps2'] <= .55 and m['spatial_min_acceleration_mps2'] >= -.55 and
                abs(m['terminal_speed_mps']) < 1e-6 for m in sample_metrics)
            (run / 'trajectory_samples.json').write_text(json.dumps(sample_metrics, indent=2) + '\n')
            if sample_metrics:
                metrics.update(
                    sampled_max_speed_mps=max(m['max_speed_mps'] for m in sample_metrics),
                    sampled_min_speed_mps=min(m['min_speed_mps'] for m in sample_metrics),
                    sampled_reported_max_acceleration_mps2=max(m['max_acceleration_mps2'] for m in sample_metrics),
                    sampled_reported_min_acceleration_mps2=min(m['min_acceleration_mps2'] for m in sample_metrics),
                    sampled_spatial_max_acceleration_mps2=max(m['spatial_max_acceleration_mps2'] for m in sample_metrics),
                    sampled_spatial_min_acceleration_mps2=min(m['spatial_min_acceleration_mps2'] for m in sample_metrics),
                )
            msg = final_samples[-1]
            points = msg.points
            checks['trajectory_nonempty'] = len(points) > 10
            if len(points) > 2:
                xyz, q, ds, curvature = geometry(points)
                speed = np.array([p.longitudinal_velocity_mps for p in points])
                accel = np.array([p.acceleration_mps2 for p in points])
                implied_accel = np.diff(speed ** 2) / (2 * np.maximum(ds, 1e-15))
                route_xy = np.array([[p.pose.position.x, p.pose.position.y]
                                     for p in samples['route_path'].poses])
                segment_start, segment_delta = route_xy[:-1], np.diff(route_xy, axis=0)
                segment_len = np.linalg.norm(segment_delta, axis=1)
                projection = np.clip(
                    np.sum((xyz[:, None, :2] - segment_start) * segment_delta, axis=2) /
                    np.maximum(segment_len ** 2, 1e-15), 0, 1)
                distances = np.linalg.norm(
                    xyz[:, None, :2] - segment_start - projection[:, :, None] * segment_delta,
                    axis=2)
                nearest = distances.argmin(axis=1)
                route_deviation = distances[np.arange(len(xyz)), nearest]
                projected_s = np.r_[0, np.cumsum(segment_len)][nearest] + projection[np.arange(len(xyz)), nearest] * segment_len[nearest]
                interior = (projected_s > 0.5) & (projected_s < segment_len.sum() - 0.5)
                start = samples['route'].start_pose.position
                goal = samples['route'].goal_pose.position
                start_error = float(np.linalg.norm(xyz[0] - [start.x, start.y, start.z]))
                goal_error = float(np.linalg.norm(xyz[-1] - [goal.x, goal.y, goal.z]))
                stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in final_samples]
                checks.update({
                    'finite_positions_orientations_profiles': bool(np.isfinite(xyz).all() and np.isfinite(q).all() and np.isfinite(speed).all() and np.isfinite(accel).all()),
                    'quaternions_normalized': bool(np.max(np.abs(np.linalg.norm(q, axis=1) - 1)) < 1e-4),
                    'trajectory_map_frame': all(m.header.frame_id == 'map' for m in final_samples),
                    'positive_continuous_spacing': bool(ds.min() > 1e-5 and ds.max() < 1.5),
                    'forward_point_ordering': bool(np.all(np.diff(xyz[:, 0]) >= -1e-5)),
                    'plausible_trajectory_length': bool(abs(ds.sum() - selected.get('route_length_m', 0)) < 1.5),
                    'curvature_within_synthetic_vehicle_limit': bool(curvature.max() <= np.tan(.45) / 1.2),
                    'nonnegative_speed_bounded_1mps': bool(speed.min() >= -1e-6 and speed.max() <= 1.0001),
                    'reported_acceleration_deceleration_bounds': bool(accel.min() >= -.53 and accel.max() <= .53),
                    'spatial_acceleration_deceleration_bounds': bool(implied_accel.min() >= -.55 and implied_accel.max() <= .55),
                    'start_goal_consistency': start_error < 1.0 and goal_error < .25,
                    'route_alignment': bool(route_deviation.max() < .65 and
                                            (not interior.any() or route_deviation[interior].max() < .15)),
                    'route_arclength_ordering': bool(np.all(np.diff(projected_s) >= -0.05)),
                    'terminal_stop': abs(speed[-1]) < 1e-6,
                    'no_unexplained_interior_stop': bool(np.all(speed[:-1][np.cumsum(np.r_[0, ds])[:-1] < ds.sum() - 1] > 1e-6)),
                    'timestamps_nonzero_advancing': bool(stamps[0] > 0 and np.all(np.diff(stamps) > 0)),
                    'conversion_smoothing_preserve_input_stamps': all((m.header.stamp.sec, m.header.stamp.nanosec) in path_stamps & raw_stamps for m in final_samples),
                })
                metrics.update({
                    'trajectory_points': len(points), 'trajectory_length_m': float(ds.sum()),
                    'max_curvature_per_m': float(curvature.max()), 'max_speed_mps': float(speed.max()),
                    'min_speed_mps': float(speed.min()), 'reported_acceleration_max_mps2': float(accel.max()),
                    'reported_deceleration_min_mps2': float(accel.min()),
                    'spatial_acceleration_max_mps2': float(implied_accel.max()),
                    'spatial_deceleration_min_mps2': float(implied_accel.min()),
                    'min_spacing_m': float(ds.min()), 'max_spacing_m': float(ds.max()),
                    'start_error_m': start_error, 'goal_error_m': goal_error,
                    'max_route_deviation_m': float(route_deviation.max()),
                    'max_interior_route_deviation_m': float(route_deviation[interior].max()) if interior.any() else None,
                    'start_speed_mps': float(speed[0]), 'terminal_speed_mps': float(speed[-1]),
                    'raw_max_speed_mps': max(p.longitudinal_velocity_mps for p in samples['raw'].points),
                    'samples_checked': len(final_samples), 'sample_interval_min_sec': float(np.diff(stamps).min() / 1e9),
                    'time_from_start_all_zero': all(p.time_from_start.sec == 0 and p.time_from_start.nanosec == 0 for p in points),
                })
                with (run / 'trajectory.csv').open('w') as f:
                    writer = csv.writer(f)
                    writer.writerow(['x','y','z','qx','qy','qz','qw','velocity_mps','acceleration_mps2','time_from_start_sec'])
                    for pt in points:
                        pose = pt.pose
                        writer.writerow([pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,
                                         pose.orientation.z,pose.orientation.w,pt.longitudinal_velocity_mps,pt.acceleration_mps2,
                                         pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9])
    text = log.read_text()
    checks['no_critical_runtime_failures'] = '[ERROR]' not in text and 'FAIL:' not in text and 'process has died' not in text
    checks['original_dataset_hashes_unchanged'] = all(hashlib.sha256(p.read_bytes()).hexdigest() == hashes[str(p)] for p in originals)
    checks['clean_launch_exit'] = process.returncode == 0
    checks = {name: bool(passed) for name, passed in checks.items()}
    failed_checks = [name for name, passed in checks.items() if not passed]
    result = {'passed': all(checks.values()), 'stage': 'route_fixture_only' if args.route_only else 'trajectory_generation_and_smoothing',
              'checks': checks, 'metrics': metrics, 'selected_route': selected, 'topic_first_receipt_sec': received,
              'failed_checks': failed_checks,
              'skipped_checks': ['rendered_image_inspection', 'controller_execution', 'simulator_execution',
                                 'measured_vehicle_dynamics', 'real_pcd_localization_alignment'] +
                                (['trajectory_generation', 'velocity_smoothing'] if args.route_only else []),
              'nodes': sorted(observed_nodes), 'rviz_subscriptions': sorted(subscribers),
              'results_dir': str(run), 'runtime_sec': time.monotonic() - begin, 'original_sha256': hashes,
              'visual_inspection': 'Unverified; no screenshot capture attempted.',
              'trajectory_claim': 'Not executed' if args.route_only else 'See actual component/publication checks and measured bounds.'}
    (run / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
    (root / 'latest_run.txt').write_text(str(run) + '\n')
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
