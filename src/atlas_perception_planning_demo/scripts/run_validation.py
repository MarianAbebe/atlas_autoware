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
from tf2_msgs.msg import TFMessage
from composition_interfaces.srv import ListNodes, UnloadNode


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
    capturing = True
    latest, records, graph = {}, {}, {}
    odom, controls, raw_controls, gated_controls, safety, base_safety, trajectories, clouds = [], [], [], [], [], [], [], []
    events = []
    stage_rows=[];stage_cache={};deskew_changes=[];deskew_accuracy=[];tf_rows=[]
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def receive(name, msg):
        if not capturing:
            return
        now = time.monotonic()-start
        latest[name] = msg
        records.setdefault(name, []).append(now)
        if name in ['lidar_raw','lidar_extended','lidar_deskewed','lidar_compact','cloud']:
            observe_stage(name,msg,now)
        if name=='tf':
            for tr in msg.transforms:
                if tr.child_frame_id=='atlas_closed_loop_base_link':tf_rows.append([stamp(tr.header.stamp),tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation),tr.header.frame_id])
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

    def observe_stage(name,msg,t):
        stage={'lidar_raw':'raw','lidar_extended':'extended','lidar_deskewed':'deskewed','lidar_compact':'compact','cloud':'segmented'}[name]
        fields={f.name:f for f in msg.fields}
        names=['x','y','z','intensity']
        dtype=np.dtype({'names':names,'formats':['<f4','<f4','<f4','<f4' if fields['intensity'].datatype==7 else 'u1'],
                        'offsets':[fields[n].offset for n in names],'itemsize':msg.point_step})
        data=np.frombuffer(msg.data,dtype=dtype,count=msg.width*msg.height)
        points=np.column_stack([data[n] for n in names]);stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        stage_cache[(stage,stamp_ns)]=points
        stage_rows.append({'t':t,'stamp':stamp_ns*1e-9,'stamp_ns':stamp_ns,'stage':stage,'points':len(points),
            'ground_points':int(np.count_nonzero(points[:,3]==10)), 'on_route_points':int(np.count_nonzero(points[:,3]==200)),
            'off_route_points':int(np.count_nonzero(points[:,3]==150)), 'point_step':msg.point_step,'frame':msg.header.frame_id,
            'age_sec':node.get_clock().now().nanoseconds*1e-9-stamp_ns*1e-9})
        if stage=='deskewed' and ('extended',stamp_ns) in stage_cache:
            original=stage_cache[('extended',stamp_ns)]
            if len(original)==len(points):deskew_changes.append({'t':t,'stamp_ns':stamp_ns,'max_change_m':float(np.linalg.norm(points[:,:3]-original[:,:3],axis=1).max())})
        if stage=='deskewed' and anchor is not None and ('extended',stamp_ns) in stage_cache and len(odom)>2:
            mask=points[:,3]==200
            if np.count_nonzero(mask)==315:
                state=np.array(odom);st=stamp_ns*1e-9
                px=np.interp(st,state[:,1],state[:,2]);py=np.interp(st,state[:,1],state[:,3]);angle=np.interp(st,state[:,1],state[:,5])
                cc,ss=math.cos(angle),math.sin(angle);rotation=np.array([[cc,-ss],[ss,cc]])
                x,y,z=np.meshgrid(np.linspace(-.4,.4,9),np.linspace(-.3,.3,7),np.linspace(.2,1.,5))
                ca,sa=math.cos(anchor['yaw']),math.sin(anchor['yaw'])
                truth=np.column_stack([anchor['x']+ca*x.ravel()-sa*y.ravel(),anchor['y']+sa*x.ravel()+ca*y.ravel()])
                corrected_world=points[mask,:2]@rotation.T+[px,py]
                raw_world=stage_cache[('extended',stamp_ns)][mask,:2]@rotation.T+[px,py]
                deskew_accuracy.append({'t':t,'stamp_ns':stamp_ns,'corrected_rms_m':float(np.sqrt(np.mean(np.sum((corrected_world-truth)**2,axis=1)))),
                    'raw_rms_m':float(np.sqrt(np.mean(np.sum((raw_world-truth)**2,axis=1))))})
        # Keep only a bounded recent set for pairing; raw evidence is point-count/stamp rows.
        if len(stage_cache)>100:
            for k in sorted(stage_cache,key=lambda k:k[1])[:20]:del stage_cache[k]

    mapping = {
        **{'lidar_'+n:('lidar/'+n,PointCloud2,qos_profile_sensor_data) for n in ['raw','extended','deskewed','compact']},
        'tf': ('/tf',TFMessage,100),
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
    subscriptions = [node.create_subscription(typ, topic if topic.startswith('/') else '/atlas_closed_loop/'+topic,
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
        process = subprocess.Popen(['ros2', 'launch', 'atlas_perception_planning_demo', 'perception_planning.launch.py',
                                    f'results_dir:={run}', 'launch_rviz:=false'], env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while time.monotonic()-start < timeout and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.02)
                now = time.monotonic()-start
                if now-last_graph >= 5:
                    last_graph = now
                    for topic, _, _ in mapping.values():
                        full = topic if topic.startswith('/') else '/atlas_closed_loop/'+topic
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
                        inject.publish(String(data=json.dumps(dict(anchor, mode='obstacle', variant=args.scenario))))
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
            shutdown.publish(String(data='measurement_complete'))
            for _ in range(15):
                rclpy.spin_once(node, timeout_sec=.02)
            for container in ['/atlas_closed_loop/perception_container', '/atlas_closed_loop/planning_container', '/atlas_closed_loop/sim/simulator_container', '/atlas_map_demo/map_container']:
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
    checks['synthetic_perception_only'] = graph.get('/atlas_closed_loop/obstacles/pointcloud', {}).get('publishers') == ['/atlas_closed_loop/ground_filter'] and 'objects' in latest and not latest['objects'].objects
    checks['controller_consumes_final_smoother'] = '/atlas_closed_loop/controller' in graph.get('/atlas_closed_loop/trajectory', {}).get('subscribers', [])
    raw_payloads = {tuple(c[1:]) for c in raw_controls}
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
                checks['fresh_perception_restored'] = bool(safety) and not set(['pointcloud','raw','extended','deskewed','compact']).intersection(safety[-1]['currently_stale'])
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
                    metrics['first_obstacle_cloud_receipt_sec'] = next((r['t'] for r in stage_rows if r['stage']=='segmented' and r['on_route_points']>0), None)
                    first_seg=metrics['first_obstacle_cloud_receipt_sec']
                    metrics['cloud_to_planning_response_sec'] = response_time-first_seg if response_time is not None and first_seg is not None else None
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
    bystage={name:[r for r in stage_rows if r['stage']==name] for name in ['raw','extended','deskewed','compact','segmented']}
    checks['all_perception_stages_execute']=all(len(rows)>20 for rows in bystage.values())
    checks['stage_layouts_frames']=all(r['point_step']=={'raw':26,'extended':32,'deskewed':32,'compact':16,'segmented':16}[r['stage']] and r['frame']=='atlas_closed_loop_base_link' for r in stage_rows)
    ground_reference=max((r['ground_points'] for r in bystage['raw']),default=1)
    ground_ratios=[r['ground_points']/ground_reference for r in bystage['segmented']]
    checks['ground_removed']=bool(ground_ratios) and max(ground_ratios)<=limits['ground_leak_fraction_max']
    checks['nonzero_motion_deskew']=bool(deskew_changes) and max(r['max_change_m'] for r in deskew_changes)>=limits['deskew_change_min_m']
    stamps={name:{r['stamp_ns']:r for r in rows} for name,rows in bystage.items()}
    common=set.intersection(*(set(rows) for rows in stamps.values())) if stamps else set()
    checks['original_stamps_preserved']=len(common)>20
    checks['counts_preserved_before_segmentation']=bool(common) and all(len({stamps[n][t]['points'] for n in ['raw','extended','deskewed','compact']})==1 for t in common)
    processing=[stamps['segmented'][t]['t']-stamps['raw'][t]['t'] for t in common]
    metrics['processing_latency_max_sec']=max(processing) if processing else None
    metrics['processing_latency_mean_sec']=float(np.mean(processing)) if processing else None
    checks['processing_latency']=bool(processing) and max(processing)<=limits['processing_latency_max_sec']
    metrics['max_deskew_change_m']=max((r['max_change_m'] for r in deskew_changes),default=0)
    metrics['stage_point_count_ranges']={n:[min((r['points'] for r in rows),default=0),max((r['points'] for r in rows),default=0)] for n,rows in bystage.items()}
    metrics['max_ground_leak_points']=max((r['ground_points'] for r in bystage['segmented']),default=0)
    if args.scenario=='ahead':
        occupied=[r for r in bystage['segmented'] if r['on_route_points']>0]
        checks['both_obstacles_segmented']=bool(occupied) and max(r['on_route_points'] for r in occupied)>=limits['segmented_obstacle_points_min'] and max(r['off_route_points'] for r in occupied)>=limits['segmented_obstacle_points_min']
        metrics['segmented_on_route_points_max']=max((r['on_route_points'] for r in occupied),default=0)
        metrics['segmented_off_route_points_max']=max((r['off_route_points'] for r in occupied),default=0)
    expected_publishers={'lidar/raw':'obstacle_source','lidar/extended':'livox_converter','lidar/deskewed':'distortion_corrector','lidar/compact':'compact_converter','obstacles/pointcloud':'ground_filter'}
    checks['no_perception_bypass']=all(graph.get('/atlas_closed_loop/'+topic,{}).get('publishers')==['/atlas_closed_loop/'+publisher] for topic,publisher in expected_publishers.items())
    if not fault:
        checks['fresh_stage_timestamps']=bool(stage_rows) and all(0<=r['age_sec']<limits['freshness_timeout_sec'] for r in stage_rows)
    if args.scenario=='ahead':
        moving_accuracy=[r for r in deskew_accuracy if r['raw_rms_m']>.01]
        metrics['deskew_corrected_rms_max_m']=max((r['corrected_rms_m'] for r in moving_accuracy),default=None)
        metrics['deskew_uncorrected_rms_max_m']=max((r['raw_rms_m'] for r in moving_accuracy),default=None)
        checks['deskew_improves_static_world_geometry']=len(moving_accuracy)>10 and all(r['corrected_rms_m']<limits['deskew_accuracy_max_m'] and r['corrected_rms_m']<r['raw_rms_m']*.5 for r in moving_accuracy)
    if odom and tf_rows:
        a=np.array(odom);res=[]
        for tr in tf_rows:
            i=int(np.argmin(abs(a[:,1]-tr[0])))
            if abs(a[i,1]-tr[0])<.03:res.append([math.hypot(a[i,2]-tr[1],a[i,3]-tr[2]),abs(a[i,5]-tr[3])])
        checks['tf_matches_live_odometry']=len(res)>100 and all(tr[4]=='map' for tr in tf_rows) and max(r[0] for r in res)<.03 and max(r[1] for r in res)<.03
        metrics['tf_samples_matched']=len(res);metrics['tf_xy_residual_max_m']=max(r[0] for r in res);metrics['tf_yaw_residual_max_rad']=max(r[1] for r in res)
    else:checks['tf_matches_live_odometry']=False
    (run/'deskew_accuracy.json').write_text(json.dumps(deskew_accuracy))
    (run/'perception_stages.json').write_text(json.dumps(stage_rows))
    (run/'deskew_changes.json').write_text(json.dumps(deskew_changes))
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
              'clustering_detector_executed': False, 'pinned_ground_filter_executed': checks.get('all_perception_stages_execute',False), 'real_lidar_executed': False,
              'rviz_rendering': 'unverified', 'anchor': anchor}
    (run/'validation.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({'recording': str(run), **report}, indent=2), flush=True)
    node.destroy_node(); rclpy.try_shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
