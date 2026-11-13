#!/usr/bin/env python3
"""Bounded independent evaluator of the synthetic pinned localization loop."""
import argparse
import bisect
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from autoware_vehicle_msgs.msg import Engage
from autoware_control_msgs.msg import Control
from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped

from measurement_cadence import MeasurementCadence

NS = '/atlas_loc/'
ROOT = Path(__file__).resolve().parents[3]


def stamp(t):
    return t.sec + t.nanosec * 1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def state(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, yaw(q)]


def expected_input_became_stale(statuses, injected, expected):
    """Cascading localization faults may latch NDT before EKF odometry ages out.

    Require the injected stream's own stale observation within the unchanged
    0.4 s evaluation bound; do not assume it wins the first-latch race.
    """
    return injected is not None and any(injected <= s['t'] < injected+.4
        and expected in s.get('currently_stale', []) for s in statuses)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', choices=['nominal', 'drop_ndt', 'drop_ekf', 'no_ndt', 'no_ekf'], default='nominal')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--timeout', type=float, default=55)
    args = ap.parse_args()
    if not 15 <= args.timeout <= 90:
        ap.error('timeout must be 15..90 seconds')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('independent_localization_evaluator')
    start = time.monotonic()
    rows = {key: [] for key in ['truth', 'ekf', 'ndt', 'command', 'gate', 'scan', 'ndt_ms', 'ndt_score', 'ndt_iterations']}
    statuses, scan_samples = [], []
    valid = {'finite_states': True, 'finite_covariance': True, 'frames': True, 'quaternions': True}
    capturing = True

    def receive(name, msg):
        if not capturing:
            return
        t = time.monotonic()-start
        if name in ['truth', 'ekf', 'ndt']:
            pose = msg.pose.pose
            header = msg.header
            values = state(pose)
            speed = msg.twist.twist.linear.x if name != 'ndt' else 0.
            rows[name].append([t, stamp(header.stamp), *values, speed])
            valid['finite_states'] &= all(math.isfinite(v) for v in values+[speed])
            valid['frames'] &= header.frame_id == 'map'
            if name == 'ekf':
                valid['frames'] &= msg.child_frame_id == 'base_link'
            if name == 'truth':
                valid['frames'] &= msg.child_frame_id == 'hidden_truth_base'
            valid['finite_covariance'] &= all(math.isfinite(v) for v in msg.pose.covariance)
            valid['finite_covariance'] &= all(msg.pose.covariance[k] >= 0 for k in [0,7,14,21,28,35])
            if name == 'ekf':
                valid['finite_covariance'] &= all(math.isfinite(v) for v in msg.twist.covariance)
            q = pose.orientation
            valid['quaternions'] &= abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < 1e-4
        elif name in ['command', 'gate']:
            rows[name].append([t, stamp(msg.stamp), msg.lateral.steering_tire_angle,
                               msg.longitudinal.velocity, msg.longitudinal.acceleration,
                               msg.lateral.steering_tire_rotation_rate,
                               msg.lateral.is_defined_steering_tire_rotation_rate])
        elif name == 'scan':
            rows[name].append([t, stamp(msg.header.stamp), msg.width*msg.height])
            valid['frames'] &= msg.header.frame_id == 'synthetic_lidar'
            # Small evolving-pose evidence, not a second registration algorithm.
            fields = {f.name: f.offset for f in msg.fields}
            points = [[struct.unpack_from('<f', msg.data, i*msg.point_step+fields[k])[0]
                       for k in ['x','y','z']] for i in [0, msg.width//2, msg.width-1]]
            if not scan_samples or t-scan_samples[-1]['receipt'] > .4:
                scan_samples.append(dict(receipt=t, stamp=stamp(msg.header.stamp), points=points))
        else:
            rows[name].append([t, stamp(msg.stamp), msg.data])

    mapping = {
        'truth': ('sim/output/odometry', Odometry), 'ekf': ('ekf/ekf_odom', Odometry),
        'ndt': ('ndt/ndt_pose_with_covariance', PoseWithCovarianceStamped),
        'command': ('control_cmd', Control), 'gate': ('gated/control_cmd', Control),
        'scan': ('sensors/points', PointCloud2), 'ndt_ms': ('ndt/exe_time_ms', Float32Stamped),
        'ndt_score': ('ndt/nearest_voxel_transformation_likelihood', Float32Stamped),
        'ndt_iterations': ('ndt/iteration_num', Int32Stamped)}
    subscriptions = [node.create_subscription(typ, NS+topic, lambda m,n=name: receive(n,m),
                     qos_profile_sensor_data if name == 'scan' else 100)
                     for name,(topic,typ) in mapping.items()]
    status_sub = node.create_subscription(String, NS+'safety_status',
        lambda m: statuses.append(dict(json.loads(m.data), t=time.monotonic()-start)) if capturing else None, 100)
    boot = node.create_publisher(PoseWithCovarianceStamped, NS+'bootstrap', 10)
    engage = node.create_publisher(Engage, NS+'sim/input/engage', 10)
    sensor_fault = node.create_publisher(String, NS+'sensor_fault', 10)
    shutdown = node.create_publisher(String, NS+'test_shutdown', 10)
    clients = {name: node.create_client(SetBool, NS+name+'/trigger_node_srv') for name in ['ndt','ekf']}
    graph, events = {}, []
    engaged = injected = recovered = hold = None
    reason = 'timeout'
    rss_cadence = MeasurementCadence(.5)
    harness_work = []
    peak_rss_kib = 0
    process = None

    def spin_until(predicate, timeout):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return True
            if process and process.poll() is not None:
                return False
        return False

    def activate(name, enabled):
        if not clients[name].wait_for_service(timeout_sec=8):
            raise RuntimeError(name+' activation service missing')
        future = clients[name].call_async(SetBool.Request(data=enabled))
        if not spin_until(future.done, 5) or not future.result().success:
            raise RuntimeError(name+' activation failed')

    def endpoints(topic, kind):
        func = node.get_publishers_info_by_topic if kind == 'publishers' else node.get_subscriptions_info_by_topic
        return sorted(set(i.node_namespace.rstrip('/')+'/'+i.node_name for i in func(NS+topic)))

    error = None
    with (args.output/'runtime.log').open('w') as log:
        process = subprocess.Popen(['ros2','launch','atlas_localization_closed_loop_demo',
                                    'localization_loop.launch.py'], stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            if not spin_until(lambda: bool(rows['truth']) and boot.get_subscription_count() > 0, 12):
                raise RuntimeError('simulator/initial-pose discovery failed')
            if args.scenario != 'no_ekf':
                activate('ekf', True)
            if args.scenario != 'no_ndt':
                activate('ndt', True)
            initial = PoseWithCovarianceStamped()
            initial.header.stamp = node.get_clock().now().to_msg()
            initial.header.frame_id = 'map'
            initial.pose.pose.position.x = 100.3
            initial.pose.pose.position.y = 100.2
            initial.pose.pose.orientation.w = 1.
            initial.pose.covariance = [0.]*36
            for k,v in [(0,.25),(7,.25),(14,.0025),(21,.0006),(28,.0006),(35,.06)]:
                initial.pose.covariance[k] = v
            boot.publish(initial)  # exactly one biased seed; never repeat truth poses
            events.append(dict(kind='single_biased_bootstrap', t=time.monotonic()-start, x=100.3,y=100.2))
            last_progress = 0
            while time.monotonic()-start < args.timeout and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.01)
                now = time.monotonic()-start
                if now-last_progress >= 5:
                    print(json.dumps(dict(elapsed=now, scenario=args.scenario, counts={n:len(rows[n]) for n in ['truth','ekf','ndt']},
                        x=rows['truth'][-1][2] if rows['truth'] else None,
                        speed=rows['truth'][-1][6] if rows['truth'] else None,
                        latched=statuses[-1]['latched'] if statuses else None)),flush=True)
                    last_progress = now
                if not graph and now > 5:
                    work_start = time.monotonic()
                    for topic in ['ekf/ekf_odom','ekf/ekf_pose_with_covariance', 'sim/output/odometry',
                                  'ndt/ndt_pose_with_covariance','sensors/points','sensors/twist','gated/control_cmd']:
                        graph[topic] = {kind: endpoints(topic,kind) for kind in ['publishers','subscribers']}
                    graph['controller_subscriptions'] = [
                        topic for topic, _types in node.get_subscriber_names_and_types_by_node('controller','/atlas_loc')]
                    harness_work.append(dict(kind='graph', start=work_start-start, duration=time.monotonic()-work_start))
                # Read only the launched process tree; resource monitoring cannot alter inputs.
                if rss_cadence.due(now):
                    work_start = time.monotonic()
                    try:
                        total = 0
                        for item in Path('/proc').glob('[0-9]*/status'):
                            data = item.read_text()
                            pid = int(item.parent.name)
                            if os.getpgid(pid) == process.pid:
                                total += next((int(l.split()[1]) for l in data.splitlines() if l.startswith('VmRSS:')),0)
                        peak_rss_kib = max(peak_rss_kib,total)
                    except (OSError,ProcessLookupError):
                        pass
                    harness_work.append(dict(kind='rss', start=work_start-start, duration=time.monotonic()-work_start))
                if args.scenario.startswith('no_'):
                    if now > 14:
                        reason = 'unavailable_localization_observed'
                        break
                    continue
                if engaged is None and statuses and statuses[-1]['armed'] and len(rows['ndt']) >= 10:
                    engage.publish(Engage(stamp=node.get_clock().now().to_msg(),engage=True))
                    engaged = now
                    events.append(dict(kind='engaged_after_localization_ready',t=now))
                if engaged is None:
                    continue
                if args.scenario.startswith('drop_') and injected is None and now-engaged > 3:
                    if rows['ekf'] and rows['ekf'][-1][6] > .2:
                        injected = now
                        events.append(dict(kind=args.scenario,t=now))
                        if args.scenario == 'drop_ndt':
                            sensor_fault.publish(String(data='drop_ndt'))
                        else:
                            activate('ekf', False)
                if injected is not None:
                    if args.scenario == 'drop_ndt' and recovered is None and now-injected > 8:
                        sensor_fault.publish(String(data='live'))
                        recovered = now
                        events.append(dict(kind='scan_recovery',t=now))
                    if now-injected > 14:
                        reason = 'fault_stop_observed'
                        break
                elif args.scenario == 'nominal' and rows['truth']:
                    latest = rows['truth'][-1]
                    near = abs(latest[2]-108) < .15 and abs(latest[6]) < .001
                    if near:
                        if hold is None:
                            hold = latest
                        if latest[0]-hold[0] >= 10 and latest[1]-hold[1] >= 10:
                            reason = 'goal_hold_completed'
                            break
                    else:
                        hold = None
            launch_alive = process.poll() is None
        except Exception as exc:
            error = repr(exc)
            launch_alive = process.poll() is None
        finally:
            capturing = False
            engage.publish(Engage(stamp=node.get_clock().now().to_msg(),engage=False))
            shutdown.publish(String(data='measurement_complete'))
            for _ in range(10):
                rclpy.spin_once(node,timeout_sec=.01)
            cleanup_start = time.monotonic()
            if process.poll() is None:
                os.kill(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    process.wait(timeout=8)
            cleanup_sec = time.monotonic()-cleanup_start

    # Independent time-aligned truth evaluation; truth never feeds control.
    truth = rows['truth']
    truth_stamps = [r[1] for r in truth]
    def matched(sample):
        i = bisect.bisect_left(truth_stamps,sample[1])
        candidates = truth[max(0,i-1):min(len(truth),i+1)]
        if not candidates:
            return None
        result = min(candidates,key=lambda r: abs(r[1]-sample[1]))
        return result if abs(result[1]-sample[1]) < .025 else None
    errors = {}
    for name in ['ndt','ekf']:
        pairs = [(r,matched(r)) for r in rows[name] if engaged is not None and r[0] >= engaged]
        pairs = [(a,b) for a,b in pairs if b is not None]
        xy = [math.hypot(a[2]-b[2],a[3]-b[3]) for a,b in pairs]
        angle = [abs(math.remainder(a[5]-b[5],2*math.pi)) for a,b in pairs]
        errors[name] = dict(matched=len(pairs), max_xy_m=max(xy,default=None),
            rms_xy_m=math.sqrt(sum(v*v for v in xy)/len(xy)) if xy else None,
            max_yaw_rad=max(angle,default=None))
    truth_moving = [r for r in truth if engaged is not None and r[0] >= engaged]
    world_samples = []
    for sample in scan_samples:
        reference = matched([0, sample['stamp']])
        if reference is None or abs(reference[1]-sample["stamp"])>1e-6:
            continue
        c, sn = math.cos(reference[5]), math.sin(reference[5])
        world_samples.append([[reference[2]+c*x-sn*y, reference[3]+sn*x+c*y, reference[4]+z]
                              for x,y,z in sample['points']])
    world_residual = max((math.dist(a,b) for sample in world_samples
                          for a,b in zip(sample,world_samples[0])),default=None)
    scan_change = max((math.dist(a,b) for sample in scan_samples
                       for a,b in zip(sample['points'],scan_samples[0]['points'])),default=0)
    accelerations = [(b[6]-a[6])/(b[1]-a[1]) for a,b in zip(truth,truth[1:]) if b[1]>a[1]]
    distances = [math.hypot(b[2]-a[2],b[3]-a[3]) for a,b in zip(truth,truth[1:])]
    checks = dict(valid, no_runtime_exception=error is None, launch_alive=launch_alive,
                  bounded_cleanup=cleanup_sec < 20,
                  controller_odom_exclusively_ekf=graph.get('ekf/ekf_odom',{}).get('publishers') == ['/atlas_loc/ekf/ekf_localizer']
                    and '/atlas_loc/controller' in graph.get('ekf/ekf_odom',{}).get('subscribers',[]),
                  truth_hidden_from_controller=not any(n in graph.get('sim/output/odometry',{}).get('subscribers',[])
                    for n in ['/atlas_loc/controller','/atlas_loc/gate/vehicle_cmd_gate','/atlas_loc/ekf/ekf_localizer','/atlas_loc/ndt/ndt_scan_matcher']),
                  ndt_pose_exclusive=graph.get('ndt/ndt_pose_with_covariance',{}).get('publishers') == ['/atlas_loc/ndt/ndt_scan_matcher'],
                  ekf_receives_ndt='/atlas_loc/ekf/ekf_localizer' in graph.get('ndt/ndt_pose_with_covariance',{}).get('subscribers',[]),
                  ndt_predictions_exclusively_ekf=graph.get('ekf/ekf_pose_with_covariance',{}).get('publishers') == ['/atlas_loc/ekf/ekf_localizer'],
                  speed_limit=all(-.05 <= r[6] <= .8 for r in truth),
                  actual_simulator_samples=len(truth)>100,
                  scans_exclusive_fixture=graph.get('sensors/points',{}).get('publishers') == ['/atlas_loc/synthetic_sensor_fixture'],
                  ndt_receives_generated_scans='/atlas_loc/ndt/ndt_scan_matcher' in graph.get('sensors/points',{}).get('subscribers',[]),
                  simulator_exclusive_gate=graph.get('gated/control_cmd',{}).get('publishers') == ['/atlas_loc/gate/vehicle_cmd_gate'],
                  scan_world_consistency=world_residual is not None and world_residual < .001,
                  finite_scan_samples=all(math.isfinite(x) for sample in scan_samples for p in sample['points'] for x in p),
                  bounded_control=all(all(math.isfinite(v) for v in r[2:6]) and abs(r[2])<=.50 and abs(r[4])<=.501 and abs(r[5])<=.6001 for r in rows['command']),
                  acceleration_limit=bool(accelerations) and max(map(abs,accelerations))<.55)
    checks['timestamps_nondecreasing'] = all(all(b[1]>=a[1] for a,b in zip(data,data[1:])) for data in rows.values())
    checks['timestamps_positive'] = all(r[1]>0 for data in rows.values() for r in data)
    if args.scenario.startswith('no_'):
        checks.update(no_engagement=engaged is None, no_truth_motion=sum(distances)<.005,
                      no_localization_authorization=not any(s['armed'] for s in statuses))
        checks['required_output_absent'] = not rows['ndt' if args.scenario=='no_ndt' else 'ekf']
    else:
        checks.update(localization_engaged=engaged is not None, actual_motion=sum(distances)>.5, scans_evolve=scan_change>.3,
            matched_ndt=len(rows['ndt'])>=10 and all(any(abs(r[1]-c[1])<1e-6 for c in rows['scan']) for r in rows['ndt']),
            ndt_convergence=bool(rows['ndt_score']) and min(r[2] for r in rows['ndt_score'])>=2.3
                and max(r[2] for r in rows['ndt_iterations'])<30,
            ndt_accuracy=errors['ndt']['matched']>10 and errors['ndt']['max_xy_m']<.10,
            ekf_accuracy=errors['ekf']['matched']>100 and errors['ekf']['max_xy_m']<.15,
            lateral_tracking=max((abs(r[3]-100) for r in truth_moving),default=100)<.3)
        if args.scenario == 'nominal':
            checks.update(goal_and_ten_second_hold=reason=='goal_hold_completed',
                          nominal_not_latched=not any(s['latched'] for s in statuses),
                          terminal_no_creep=bool(hold) and max((math.dist(r[2:4],hold[2:4]) for r in truth if r[0]>=hold[0]),default=100)<.005,
                          terminal_commands=bool(hold) and all(abs(r[3])<.001 and r[4]<=0 for r in rows['command'] if r[0]>=hold[0]+.5))
        else:
            fault = next((s for s in statuses if s['latched'] and injected is not None and s['t']>=injected),None)
            stopping = [r for r in truth if injected is not None and r[0]>=injected]
            tail = [r for r in truth if stopping and r[0]>=stopping[-1][0]-5]
            stopped = next((r for r in stopping if abs(r[6])<.001),None)
            checks.update(fault_injected=injected is not None, expected_latch=bool(fault) and
                expected_input_became_stale(statuses, injected, 'ndt' if args.scenario=='drop_ndt' else 'odometry'),
                no_unrequested_latch_before_injection=injected is not None and not any(s['latched'] and s['t']<injected for s in statuses),
                latch_latency=bool(fault) and fault['t']-injected<.4,
                bounded_stop=bool(stopped) and math.hypot(stopped[2]-stopping[0][2],stopped[3]-stopping[0][3])<1,
                final_stopped=bool(tail) and all(abs(r[6])<.001 for r in tail),
                no_auto_rearm=bool(fault) and all(s['latched'] for s in statuses if s['t']>=fault['t']),
                completed_fault_observation=reason=='fault_stop_observed')
    metrics = dict(scenario=args.scenario, termination=reason, error=error, engaged=engaged,
        injected=injected, cleanup_sec=cleanup_sec, process_tree_peak_rss_kib=peak_rss_kib,
        localization_errors=errors, counts={n:len(r) for n,r in rows.items()},
        truth_distance_m=sum(distances), final_goal_distance_m=math.hypot(truth[-1][2]-108,truth[-1][3]-100) if truth else None,
        max_lateral_error_m=max((abs(r[3]-100) for r in truth),default=None),
        max_speed_mps=max((r[6] for r in truth),default=None),
        max_command_steering_rad=max((abs(r[2]) for r in rows['command']),default=None),
        max_command_steering_rate_radps=max((abs(r[5]) for r in rows['command']),default=None),
        scan_world_residual_m=world_residual, scan_world_matched=len(world_samples), scan_sample_displacement_m=scan_change,
        max_acceleration_mps2=max(map(abs,accelerations),default=None),
        ndt_execution_ms_max=max((r[2] for r in rows['ndt_ms']),default=None),
        ndt_score_min=min((r[2] for r in rows['ndt_score']),default=None),
        ndt_iterations_max=max((r[2] for r in rows['ndt_iterations']),default=None))
    for name,data in rows.items():
        with (args.output/(name+'.csv')).open('w') as stream:
            csv.writer(stream).writerows(data)
    for name,data in [('graph',graph),('events',events),('safety',statuses),('scan_samples',scan_samples),('harness_work',harness_work)]:
        (args.output/(name+'.json')).write_text(json.dumps(data,indent=2))
    files = list((ROOT/'src/atlas_localization_closed_loop_demo').rglob('*.py')) + list((ROOT/'src/atlas_localization_closed_loop_demo').rglob('*.cpp'))
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    installed = [Path(get_package_share_directory('atlas_localization_closed_loop_demo'))/'launch/localization_loop.launch.py',
                 Path(get_package_prefix('atlas_localization_closed_loop_demo'))/'lib/atlas_localization_closed_loop_demo/localization_watchdog.py']
    (args.output/'installed_sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in installed},indent=2))
    (args.output/'source_sha256.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
    result = dict(passed=all(checks.values()), checks=checks, failed_checks=[n for n,v in checks.items() if not v], metrics=metrics)
    (args.output/'validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
