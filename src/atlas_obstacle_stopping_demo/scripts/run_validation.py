#!/usr/bin/env python3
"""Bounded simulator measurements. Only perception/fault fixtures are injected."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from autoware_planning_msgs.msg import Trajectory, LaneletRoute
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import Engage, SteeringReport
from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from composition_interfaces.srv import ListNodes, UnloadNode


import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'atlas_closed_loop_demo/scripts'))
from command_recording import recording_barrier

def stamp(s):
    return s.sec + s.nanosec * 1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def rectangle(x, y, angle, rear, front, half_width):
    shape = np.array([[-rear, -half_width], [front, -half_width],
                      [front, half_width], [-rear, half_width]])
    c, s = math.cos(angle), math.sin(angle)
    return shape @ np.array([[c, s], [-s, c]]) + [x, y]


def clearance(row, anchor):
    """Exact planar distance between provisional vehicle and obstacle rectangles."""
    a = rectangle(row[2], row[3], row[5], .3, 1.5, .5)
    b = rectangle(anchor['x'], anchor['y'], anchor['yaw'], .4, .4, .3)
    separated = False
    for shape in (a, b):
        for edge in np.roll(shape, -1, axis=0)-shape:
            axis = np.array([-edge[1], edge[0]])
            pa, pb = a@axis, b@axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                separated = True
    if not separated:
        return 0.0
    distances = []
    for points, poly in ((a, b), (b, a)):
        edges = np.roll(poly, -1, axis=0)-poly
        for point in points:
            u = np.clip(np.sum((point-poly)*edges, axis=1)/np.sum(edges*edges, axis=1), 0, 1)
            distances.append(np.linalg.norm(poly+u[:, None]*edges-point, axis=1).min())
    return float(min(distances))


def tracking(rows, poly):
    d = np.diff(poly, axis=0)
    valid = np.sum(d*d, axis=1) > 1e-10
    starts, d = poly[:-1][valid], d[valid]
    errors = []
    for row in rows:
        rel = row[2:4]-starts
        u = np.clip(np.sum(rel*d, axis=1)/np.sum(d*d, axis=1), 0, 1)
        idx = np.argmin(np.linalg.norm(starts+u[:, None]*d-row[2:4], axis=1))
        errors.append(abs(d[idx, 0]*rel[idx, 1]-d[idx, 1]*rel[idx, 0])/np.linalg.norm(d[idx]))
    return float(max(errors)), float(np.sqrt(np.mean(np.square(errors))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', choices=['none', 'ahead', 'off_route', 'stale_drop', 'stale_freeze', 'feedback_drop', 'feedback_freeze'], default='ahead')
    ap.add_argument('--feedback-input', choices=['odometry', 'steering', 'command'], default='odometry')
    ap.add_argument('--timeout', type=float)
    ap.add_argument('--results-root', type=Path, default=Path(__file__).resolve().parents[1]/'results/runs')
    args = ap.parse_args()
    demo = Path(__file__).resolve().parents[1]
    limits = json.loads((demo/'config/thresholds.json').read_text())
    fault = args.scenario.startswith(('stale_', 'feedback_'))
    timeout = args.timeout or limits['stale_test_timeout_sec' if fault else 'obstacle_clearance_test_timeout_sec' if args.scenario == 'ahead' else 'normal_test_timeout_sec']
    run = args.results_root/(time.strftime('run_%Y%m%d_%H%M%S')+'_'+args.scenario+'_'+args.feedback_input)
    run.mkdir(parents=True)
    (run/'thresholds.json').write_text(json.dumps(limits, indent=2))
    hashes = {str(p.relative_to(demo)): hashlib.sha256(p.read_bytes()).hexdigest()
              for folder in ['config', 'launch', 'scripts'] for p in (demo/folder).glob('*') if p.is_file()}
    (run/'source_sha256.json').write_text(json.dumps(hashes, indent=2))
    reused = demo.parent/'atlas_closed_loop_demo'
    (run/'reused_source_sha256.json').write_text(json.dumps({str(p.relative_to(reused)):
        hashlib.sha256(p.read_bytes()).hexdigest() for folder in ['config', 'launch', 'scripts']
        for p in (reused/folder).glob('*') if p.is_file()}, indent=2))
    originals = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                 (Path(os.environ.get('ATLAS_MAP_DIR', Path.home()/'Downloads')).expanduser()/name for name in ['finalMap_repaired.osm', 'map_projector_info.yaml', 'pointcloud_map_georeferenced.pcd'])}
    rclpy.init()
    node = rclpy.create_node('obstacle_validation')
    start = time.monotonic()
    command_recording_client = node.create_client(Trigger, '/atlas_closed_loop/command_recording/snapshot')
    recording_result = dict(passed=False, reason='not_completed', rows=[])
    capturing = True
    latest, records, graph = {}, {}, {}
    odom, controls, raw_controls, gated_controls, safety, base_safety, trajectories, clouds = [], [], [], [], [], [], [], []
    events = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def receive(name, msg):
        if not capturing:
            return
        now = time.monotonic()-start
        latest[name] = msg
        records.setdefault(name, []).append(now)
        if name == 'odom':
            p = msg.pose.pose.position
            odom.append([now, stamp(msg.header.stamp), p.x, p.y, p.z, yaw(msg.pose.pose.orientation), msg.twist.twist.linear.x])
        elif name in ('control', 'raw_control', 'gated_control'):
            target = {'control': controls, 'raw_control': raw_controls, 'gated_control': gated_controls}[name]
            target.append([now, stamp(msg.stamp), msg.lateral.steering_tire_angle, msg.longitudinal.velocity, msg.longitudinal.acceleration])
        elif name in ('safety', 'base_safety', 'source'):
            data = dict(json.loads(msg.data), t=now)
            if name == 'safety': safety.append(data)
            elif name == 'base_safety': base_safety.append(data)
            else: events.append(dict(data, kind='source'))
        elif name in ('raw', 'modified', 'smooth'):
            # Retain full sparse trajectories, with original stamps and velocities.
            points = [[p.pose.position.x, p.pose.position.y, p.pose.position.z,
                       p.longitudinal_velocity_mps] for p in msg.points]
            trajectories.append({'t': now, 'kind': name, 'stamp': stamp(msg.header.stamp), 'points': points})
            if name == 'smooth' and 'initial' not in latest and len(msg.points) > 150:
                latest['initial'] = msg
        elif name == 'cloud':
            clouds.append([now, stamp(msg.header.stamp), msg.width*msg.height, msg.header.frame_id])
            if msg.width*msg.height and 'cloud_snapshot' not in latest:
                latest['cloud_snapshot'] = {'receipt_sec': now, 'stamp': stamp(msg.header.stamp),
                    'frame': msg.header.frame_id, 'fields': [[f.name, f.offset, f.datatype] for f in msg.fields],
                    'points': [list(map(float, p)) for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'))]}

    mapping = {
        'odom': ('sim/output/odometry', Odometry, 10),
        'feedback': ('feedback/odometry', Odometry, 10),
        'steering': ('feedback/steering', SteeringReport, 10),
        'control': ('control_cmd', Control, 10),
        'raw_control': ('controller_raw/control_cmd', Control, 10),
        'gated_control': ('gated/control_cmd', Control, 10),
        'safety': ('safety_status', String, 10),
        'base_safety': ('base_safety_status', String, 10),
        'source': ('obstacles/source_status', String, 10),
        'cloud': ('obstacles/pointcloud', PointCloud2, qos_profile_sensor_data),
        'objects': ('obstacles/predicted_objects', PredictedObjects, 10),
        'raw': ('raw_trajectory', Trajectory, 10),
        'modified': ('obstacle_trajectory', Trajectory, 10),
        'smooth': ('trajectory', Trajectory, 10),
        'route': ('route', LaneletRoute, latched),
        'route_path': ('route_path', RosPath, latched),
    }
    subscriptions = [node.create_subscription(typ, '/atlas_closed_loop/'+topic,
        lambda m, name=name: receive(name, m), qos) for name, (topic, typ, qos) in mapping.items()]
    inject = node.create_publisher(String, '/atlas_closed_loop/obstacles/inject', 10)
    feedback_inject = node.create_publisher(String, '/atlas_closed_loop/inject_feedback', 10)
    shutdown = node.create_publisher(String, '/atlas_closed_loop/test_shutdown', 10)
    pause = node.create_publisher(Engage, '/atlas_closed_loop/sim/input/engage', 10)
    guard_rearm = node.create_client(Trigger, '/atlas_closed_loop/obstacle_guard/rearm')
    base_rearm = node.create_client(Trigger, '/atlas_closed_loop/feedback_watchdog/rearm')
    rearm_futures = []
    injected = removed = stop_begin = goal_stop_begin = None
    anchor = None
    response_time = None
    stop_sample = None
    reason = 'timeout'
    last_graph = last_progress = 0
    env = dict(os.environ, ROS_LOG_DIR=str(run/'ros_logs'), RCUTILS_COLORIZED_OUTPUT='0')
    with (run/'runtime.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'atlas_obstacle_stopping_demo', 'obstacle_stopping.launch.py', 'record_commands:=true',
                                    f'results_dir:={run}', 'launch_rviz:=false'], env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while time.monotonic()-start < timeout and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.02)
                now = time.monotonic()-start
                if now-last_graph >= 5:
                    last_graph = now
                    for topic, _, _ in mapping.values():
                        full = '/atlas_closed_loop/'+topic
                        graph[full] = {'publishers': [i.node_namespace.rstrip('/')+'/'+i.node_name for i in node.get_publishers_info_by_topic(full)],
                                       'subscribers': [i.node_namespace.rstrip('/')+'/'+i.node_name for i in node.get_subscriptions_info_by_topic(full)]}
                if not odom or 'route' not in latest or 'route_path' not in latest:
                    continue
                row = odom[-1]
                route = latest['route']
                distance = math.hypot(row[2]-route.goal_pose.position.x, row[3]-route.goal_pose.position.y)
                travel_from_start = math.hypot(row[2]-route.start_pose.position.x, row[3]-route.start_pose.position.y)
                if now-last_progress > 10:
                    last_progress = now
                    print(json.dumps({'elapsed': now, 'scenario': args.scenario, 'speed': row[6], 'goal_m': distance,
                                      'active_obstacle_clearance_m': clearance(row, anchor) if anchor and (args.scenario == 'off_route' or removed is None) else None,
                                      'latched': safety[-1]['latched'] if safety else None}), flush=True)
                if args.scenario != 'none' and injected is None and row[6] >= .5 and travel_from_start >= 5:
                    injected = now
                    if fault:
                        if args.scenario.startswith('stale_'):
                            inject.publish(String(data=json.dumps({'mode': args.scenario.split('_')[1]})))
                        else:
                            feedback_inject.publish(String(data=args.scenario.split('_')[1]+':'+args.feedback_input))
                    else:
                        path = latest['route_path'].poses
                        xyz = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path])
                        lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1))]
                        idx = int(np.argmin(abs(lengths-20)))
                        tangent = xyz[min(idx+1, len(xyz)-1)]-xyz[max(idx-1, 0)]
                        angle = math.atan2(tangent[1], tangent[0])
                        lateral = limits['off_route_obstacle_lateral_center_offset_m'] if args.scenario == 'off_route' else 0
                        anchor = {'x': float(xyz[idx, 0]-math.sin(angle)*lateral),
                                  'y': float(xyz[idx, 1]+math.cos(angle)*lateral),
                                  'z': float(xyz[idx, 2]), 'yaw': angle}
                        inject.publish(String(data=json.dumps(dict(anchor, mode='obstacle'))))
                    events.append({'kind': 'injection_request', 't': now, 'ros_stamp': stamp(node.get_clock().now().to_msg()),
                                   'scenario': args.scenario, 'anchor': anchor, 'odometry': row})
                if injected is not None and args.scenario == 'ahead' and removed is None:
                    # A new zero point far before the goal identifies the planning decision.
                    for t in reversed(trajectories[-20:]):
                        if t['kind'] == 'modified' and t['t'] >= injected:
                            zeros = [p for p in t['points'] if abs(p[3]) < 1e-5 and
                                     math.hypot(p[0]-route.goal_pose.position.x, p[1]-route.goal_pose.position.y) > 5 and
                                     math.hypot(p[0]-row[2], p[1]-row[3]) < 20]
                            if zeros and response_time is None:
                                response_time = t['t']
                    if abs(row[6]) < limits['stopped_speed_abs_max_mps'] and travel_from_start > 7:
                        if stop_begin is None:
                            stop_begin = row[0]
                            stop_sample = row.copy()
                        if row[0]-stop_begin >= limits['obstacle_hold_min_sec']+.05 and row[1]-stop_sample[1] >= limits['obstacle_hold_min_sec']+.05:
                            inject.publish(String(data=json.dumps({'mode': 'clear'})))
                            removed = now
                            events.append({'kind': 'clearance_request', 't': now, 'odometry': row})
                    else:
                        stop_begin = None
                if injected is not None and fault:
                    if now-injected >= 2 and not rearm_futures:
                        for client in ([guard_rearm, base_rearm] if args.scenario.startswith('feedback_') else [guard_rearm]):
                            if client.service_is_ready():
                                rearm_futures.append(client.call_async(Trigger.Request()))
                    if now-injected >= 8 and removed is None:
                        if args.scenario.startswith('stale_'):
                            inject.publish(String(data=json.dumps({'mode': 'clear'})))
                        else:
                            feedback_inject.publish(String(data='live:'+args.feedback_input))
                        removed = now
                        events.append({'kind': 'recovery_request', 't': now})
                    if removed is not None and now-removed >= limits['fault_recovery_observation_min_sec']+.1:
                        reason = 'fault_recovery_observed'
                        break
                elif travel_from_start > 60 and distance < limits['final_goal_distance_m'] and abs(row[6]) < limits['stopped_speed_abs_max_mps']:
                    if goal_stop_begin is None:
                        goal_stop_begin = row.copy()
                    if row[0]-goal_stop_begin[0] >= limits['terminal_hold_min_sec']+.05 and row[1]-goal_stop_begin[1] >= limits['terminal_hold_min_sec']+.05:
                        reason = 'terminal_hold_completed'
                        break
                else:
                    goal_stop_begin = None
                if abs(row[6]) > 1.3 or row[6] < -.05:
                    reason = 'unsafe_speed_abort'
                    break
                if anchor and args.scenario == 'ahead' and removed is None and clearance(row, anchor) < .1:
                    reason = 'clearance_abort'
                    break
                if now > 30 and travel_from_start < .1:
                    reason = 'no_motion_abort'
                    break
            alive = process.poll() is None
        finally:
            end = time.monotonic()
            capturing = False
            log.write('\nATLAS_MEASUREMENT_COMPLETE\n')
            log.flush()
            # Cleanup pause only after measurement closes; cannot pass a stop check.
            pause.publish(Engage(stamp=node.get_clock().now().to_msg(), engage=False))
            recording_result = recording_barrier(node, command_recording_client, controls)
            (run/'command_recording.json').write_text(json.dumps(recording_result, indent=2))
            shutdown.publish(String(data='measurement_complete'))
            for _ in range(15):
                rclpy.spin_once(node, timeout_sec=.02)
            for container in ['/atlas_closed_loop/planning_container', '/atlas_closed_loop/sim/simulator_container', '/atlas_map_demo/map_container']:
                client = node.create_client(ListNodes, container+'/_container/list_nodes')
                if client.service_is_ready():
                    future = client.call_async(ListNodes.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=1)
                    if future.done() and future.result() is not None:
                        unload = node.create_client(UnloadNode, container+'/_container/unload_node')
                        for unique_id in future.result().unique_ids:
                            req = UnloadNode.Request(unique_id=unique_id)
                            done = unload.call_async(req)
                            rclpy.spin_until_future_complete(node, done, timeout_sec=1)
                        node.destroy_client(unload)
                node.destroy_client(client)
            if process.poll() is None:
                os.kill(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
    cleanup = time.monotonic()-end
    checks = {'launch_alive_during_measurement': alive, 'bounded_cleanup': cleanup <= limits['shutdown_timeout_sec'] and process.returncode == 0,
              'no_map_changes': all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p, h in originals.items()),
              'actual_motion': len(odom) > 100 and max((r[6] for r in odom), default=0) > .4,
              'actual_planner_output': len(records.get('modified', [])) > 20,
              'actual_controller_feedback': len(records.get('feedback', [])) > 100 and len(records.get('steering', [])) > 100}
    checks['pinned_planner_exclusive_output'] = graph.get('/atlas_closed_loop/obstacle_trajectory', {}).get('publishers') == ['/atlas_closed_loop/motion_velocity_planner']
    checks['pinned_gate_exclusive_simulator_command'] = graph.get('/atlas_closed_loop/gated/control_cmd', {}).get('publishers') == ['/atlas_closed_loop/gate/vehicle_cmd_gate']
    checks['synthetic_perception_only'] = graph.get('/atlas_closed_loop/obstacles/pointcloud', {}).get('publishers') == ['/atlas_closed_loop/obstacle_source'] and 'objects' in latest and not latest['objects'].objects
    checks['controller_consumes_final_smoother'] = '/atlas_closed_loop/controller' in graph.get('/atlas_closed_loop/trajectory', {}).get('subscribers', [])
    checks['command_recording_barrier'] = recording_result['passed']
    raw_payloads = {tuple(c[1:]) for c in recording_result['rows']}
    checks['delivered_commands_match_pinned_controller'] = bool(controls) and all(tuple(c[1:]) in raw_payloads for c in controls)
    metrics = {'scenario': args.scenario, 'feedback_input': args.feedback_input, 'termination_reason': reason,
               'injection_request_sec': injected, 'clearance_or_recovery_request_sec': removed, 'shutdown_sec': cleanup}
    if len(odom) > 2 and 'route' in latest and 'initial' in latest:
        a = np.array(odom)
        segment = np.linalg.norm(np.diff(a[:, 2:4], axis=0), axis=1)
        cumulative = np.r_[0, np.cumsum(segment)]
        dt = np.diff(a[:, 1]); valid = dt > 1e-6
        acceleration = np.diff(a[:, 6])[valid]/dt[valid]
        initial = np.array([[p.pose.position.x, p.pose.position.y] for p in latest['initial'].points])
        max_lat, rms_lat = tracking(a, initial)
        goal = latest['route'].goal_pose.position
        metrics.update(total_distance_m=float(cumulative[-1]), final_goal_distance_m=float(np.linalg.norm(a[-1, 2:4]-[goal.x, goal.y])),
                       max_speed_mps=float(max(a[:, 6])), max_acceleration_mps2=float(max(acceleration)),
                       max_deceleration_mps2=float(min(acceleration)), final_speed_mps=float(a[-1, 6]),
                       max_lateral_error_m=max_lat, rms_lateral_error_m=rms_lat,
                       initial_trajectory_points=len(initial), initial_trajectory_length_m=float(np.linalg.norm(np.diff(initial, axis=0), axis=1).sum()),
                       controller_command_hz=(len(raw_controls)-1)/(raw_controls[-1][0]-raw_controls[0][0]) if len(raw_controls)>1 else 0)
        checks['speed_limit'] = metrics['max_speed_mps'] <= limits['max_speed_mps'] and min(a[:, 6]) >= -.001
        checks['acceleration_limit'] = max(abs(acceleration)) <= limits['max_abs_acceleration_mps2']
        checks['tracking'] = max_lat <= limits['max_lateral_error_m'] and rms_lat <= limits['rms_lateral_error_m']
        checks['validated_initial_trajectory'] = len(initial) == 226 and abs(metrics['initial_trajectory_length_m']-70.278681) < .01
        if fault and injected is not None and removed is not None:
            tail = a[a[:, 0] >= removed]
            latched_events = [s for s in (base_safety if args.scenario.startswith('feedback_') else safety) if s['t'] >= injected and s['latched']]
            stop_rows = a[(a[:, 0] >= injected) & (abs(a[:, 6]) < .001)]
            settled = stop_rows[0] if len(stop_rows) else None
            injection_index = np.searchsorted(a[:, 0], injected)
            stopped_index = np.searchsorted(a[:, 0], settled[0]) if settled is not None else len(a)-1
            metrics.update(detection_latency_sec=latched_events[0]['t']-injected if latched_events else None,
                           stopping_distance_m=float(cumulative[stopped_index]-cumulative[min(injection_index, len(a)-1)]),
                           stop_settling_sec=float(settled[0]-injected) if settled is not None else None,
                           recovery_hold_sec=float(tail[-1, 0]-tail[0, 0]) if len(tail) else 0,
                           recovery_creep_m=float(np.linalg.norm(np.diff(tail[:, 2:4], axis=0), axis=1).sum()) if len(tail) else None)
            checks['fault_detected'] = bool(latched_events) and metrics['detection_latency_sec'] <= limits['stale_detection_max_sec']
            checks['actual_fault_stop'] = settled is not None and metrics['stopping_distance_m'] <= limits['stale_stopping_distance_max_m'] and metrics['stop_settling_sec'] <= limits['stale_stop_settling_max_sec']
            checks['no_automatic_rearm'] = len(tail)>10 and max(abs(tail[:, 6])) < .001 and all(s['latched'] for s in safety if s['t'] >= removed) and metrics['recovery_creep_m'] <= limits['hold_travel_max_m']
            checks['invalid_rearm_refused'] = bool(rearm_futures) and all(f.done() and f.result() is not None and not f.result().success for f in rearm_futures)
            checks['completed_fault_observation'] = reason == 'fault_recovery_observed' and metrics['recovery_hold_sec'] >= limits['fault_recovery_observation_min_sec']
            if args.scenario.startswith('feedback_'):
                checks['fresh_feedback_restored'] = bool(base_safety) and args.feedback_input not in base_safety[-1]['currently_stale']
            if args.scenario.startswith('stale_'):
                checks['fresh_perception_restored'] = bool(safety) and 'pointcloud' not in safety[-1]['currently_stale']
        else:
            checks['completed_route'] = reason == 'terminal_hold_completed' and metrics['final_goal_distance_m'] <= limits['final_goal_distance_m']
            checks['nominal_safety_not_latched'] = bool(safety) and not any(s['latched'] for s in safety+base_safety)
            tail = a[a[:, 0] >= goal_stop_begin[0]] if goal_stop_begin is not None else a[-1:]
            metrics.update(terminal_hold_sec=float(tail[-1, 0]-tail[0, 0]), terminal_creep_m=float(np.linalg.norm(np.diff(tail[:, 2:4], axis=0), axis=1).sum()))
            checks['terminal_hold'] = metrics['terminal_hold_sec'] >= limits['terminal_hold_min_sec'] and max(abs(tail[:, 6])) < .001 and metrics['terminal_creep_m'] <= limits['hold_travel_max_m']
            if args.scenario == 'ahead':
                checks['obstacle_injected'] = injected is not None and anchor is not None
                if anchor is not None and injected is not None:
                    active = a[(a[:, 0] >= injected) & (a[:, 0] <= (removed or a[-1, 0]))]
                    stopped = a[(a[:, 0] >= (stop_begin or a[-1, 0])) & (a[:, 0] <= (removed or a[-1, 0]))]
                    metrics.update(initial_obstacle_center_distance_m=float(np.linalg.norm(active[0, 2:4]-[anchor['x'], anchor['y']])),
                                   initial_obstacle_clearance_m=clearance(active[0], anchor),
                                   minimum_obstacle_clearance_m=min(clearance(r, anchor) for r in active),
                                   stopped_obstacle_clearance_m=clearance(stopped[0], anchor),
                                   planning_response_latency_sec=response_time-injected if response_time is not None else None,
                                   obstacle_hold_sec=float(stopped[-1, 0]-stopped[0, 0]),
                                   obstacle_hold_max_speed_mps=float(max(abs(stopped[:, 6]))),
                                   obstacle_hold_creep_m=float(np.linalg.norm(np.diff(stopped[:, 2:4], axis=0), axis=1).sum()))
                    inj_idx = np.searchsorted(a[:, 0], injected)
                    stop_idx = np.searchsorted(a[:, 0], stop_begin) if stop_begin else len(a)-1
                    metrics['stopping_distance_m'] = float(cumulative[stop_idx]-cumulative[inj_idx])
                    after = a[a[:, 0] >= removed] if removed else a[-1:]
                    moving = after[after[:, 6] > limits['resumption_min_speed_mps']]
                    metrics['resumption_latency_sec'] = float(moving[0, 0]-removed) if len(moving) and removed else None
                    idx15 = np.searchsorted(a[:, 0], removed+15) if removed else len(a)-1
                    idxclear = np.searchsorted(a[:, 0], removed) if removed else len(a)-1
                    metrics['resumption_15s_travel_m'] = float(cumulative[min(idx15, len(a)-1)]-cumulative[min(idxclear, len(a)-1)])
                    checks['initial_obstacle_distance'] = limits['obstacle_initial_front_clearance_min_m'] <= metrics['initial_obstacle_clearance_m'] <= limits['obstacle_initial_front_clearance_max_m']
                    checks['actual_safe_clearance'] = metrics['minimum_obstacle_clearance_m'] >= limits['obstacle_min_clearance_during_run_m'] and limits['obstacle_stopped_front_clearance_min_m'] <= metrics['stopped_obstacle_clearance_m'] <= limits['obstacle_stopped_front_clearance_max_m']
                    checks['pinned_planning_response'] = response_time is not None and metrics['planning_response_latency_sec'] <= limits['planning_response_max_sec']
                    snapshot = latest.get('cloud_snapshot')
                    checks['received_expected_cloud_geometry'] = False
                    checks['raw_to_modified_stop_insertion'] = False
                    if snapshot:
                        points = np.array(snapshot['points'])
                        c, s = math.cos(anchor['yaw']), math.sin(anchor['yaw'])
                        relative = (points[:, :2]-[anchor['x'], anchor['y']]) @ np.array([[c, -s], [s, c]])
                        checks['received_expected_cloud_geometry'] = snapshot['frame'] == 'map' and len(points) == 252 and np.isfinite(points).all() and np.max(abs(relative), axis=0)[0] <= .41 and np.max(abs(relative), axis=0)[1] <= .31
                        metrics['first_obstacle_cloud_receipt_sec'] = snapshot['receipt_sec']
                        metrics['cloud_to_planning_response_sec'] = response_time-snapshot['receipt_sec'] if response_time is not None else None
                    if response_time is not None:
                        modified = next(t for t in trajectories if t['kind'] == 'modified' and t['t'] == response_time)
                        raw_match = next((t for t in trajectories if t['kind'] == 'raw' and abs(t['stamp']-modified['stamp']) < 1e-7), None)
                        stops = [p for p in modified['points'] if abs(p[3]) < 1e-5 and math.hypot(p[0]-goal.x, p[1]-goal.y) > 5]
                        if raw_match and stops:
                            stop_point = stops[0]
                            nearest = min(raw_match['points'], key=lambda p: math.hypot(p[0]-stop_point[0], p[1]-stop_point[1]))
                            metrics['inserted_stop_point_map'] = stop_point[:3]
                            metrics['raw_speed_near_inserted_stop_mps'] = nearest[3]
                            checks['raw_to_modified_stop_insertion'] = nearest[3] > .5 and stop_point[3] == 0
                    checks['obstacle_stop_hold'] = metrics['obstacle_hold_sec'] >= limits['obstacle_hold_min_sec'] and metrics['obstacle_hold_max_speed_mps'] < .001 and metrics['obstacle_hold_creep_m'] <= limits['hold_travel_max_m']
                    checks['normal_resumption'] = metrics['resumption_latency_sec'] is not None and metrics['resumption_latency_sec'] <= limits['resumption_max_sec_after_clearance'] and metrics['resumption_15s_travel_m'] >= limits['resumption_travel_min_m_within_15_sec']
            elif args.scenario == 'off_route':
                # Ignore the legitimate terminal braking region, not obstacle passage.
                region = a[(a[:, 0] >= (injected or a[-1, 0])) & (np.linalg.norm(a[:, 2:4]-[goal.x, goal.y], axis=1) > 5)]
                longest = ongoing = 0.0
                for i in range(1, len(region)):
                    ongoing = ongoing+region[i, 0]-region[i-1, 0] if abs(region[i, 6]) < limits['off_route_false_stop_speed_threshold_mps'] else 0
                    longest = max(longest, ongoing)
                metrics['off_route_false_stop_duration_sec'] = longest
                checks['no_off_route_false_stop'] = injected is not None and len(region)>100 and longest <= limits['off_route_false_stop_duration_max_sec']
    runtime_text = (run/'runtime.log').read_text()
    measurement_log = runtime_text.split('ATLAS_MEASUREMENT_COMPLETE')[0]
    checks['plugin_loaded'] = 'ObstacleStopModule' in measurement_log and 'Failed to load' not in measurement_log
    checks['no_crash'] = 'process has died' not in measurement_log and 'terminate called' not in measurement_log
    for name, rows, header in [('odometry', odom, 'wall_sec,stamp,x,y,z,yaw,speed'),
                                ('controller_raw', raw_controls, 'wall_sec,stamp,steering,velocity,acceleration'),
                                ('controller_delivered', controls, 'wall_sec,stamp,steering,velocity,acceleration'),
                                ('gate', gated_controls, 'wall_sec,stamp,steering,velocity,acceleration'),
                                ('clouds', clouds, 'wall_sec,stamp,points,frame')]:
        with (run/(name+'.csv')).open('w') as f:
            writer = csv.writer(f); writer.writerow(header.split(',')); writer.writerows(rows)
    for name, value in [('events', events), ('trajectories', trajectories), ('safety', safety), ('base_safety', base_safety), ('graph', graph)]:
        (run/(name+'.json')).write_text(json.dumps(value))
    if 'cloud_snapshot' in latest:
        (run/'obstacle_cloud_snapshot.json').write_text(json.dumps(latest['cloud_snapshot'], indent=2))
    report = {'passed': bool(checks) and all(checks.values()), 'checks': {k: bool(v) for k, v in checks.items()},
              'failed_checks': [k for k, v in checks.items() if not v], 'metrics': metrics,
              'original_sha256': originals, 'synthetic_perception_only': True,
              'clustering_detector_executed': False, 'real_lidar_executed': False,
              'rviz_rendering': 'unverified', 'anchor': anchor}
    (run/'validation.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({'recording': str(run), **report}, indent=2), flush=True)
    node.destroy_node(); rclpy.try_shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
