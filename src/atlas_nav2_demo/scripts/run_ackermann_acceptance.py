#!/usr/bin/env python3
"""Run bounded isolated live Nav2 acceptance trials and retain per-run evidence."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from atlas_nav2_demo.overlay_contract import is_development_package_prefix
from atlas_nav2_demo.world import clearance


# Simulation placeholder limits; these are not measured Atlas hardware values.
MAX_STEERING_RAD = 0.50
MAX_STEERING_RATE_RADPS = 0.60
MAX_SPEED_MPS = 0.80
MAX_ACCELERATION_MPS2 = 0.80
MAX_DECELERATION_MPS2 = 1.20
ROBOT_RADIUS_M = 0.32
LIMIT_TOLERANCE = 1.0e-6


def read_records(path):
    with Path(path).open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def classify(records, scenario):
    action = next((row['data'] for row in reversed(records)
                   if row['event'] == 'action_result'), None)
    if action is None:
        return False, {'reason': 'missing_action_result'}
    odometry = [row['data'] for row in records
                if row.get('topic') == '/atlas_ackermann/odom']
    safe = [row['data'] for row in records
            if row.get('topic') == '/atlas_ackermann/cmd_vel_safe']
    states = [row['data'] for row in records
              if row.get('topic') == '/atlas_nav2/safety_status']
    if scenario == 'blocked':
        moved = False
        if odometry:
            start = odometry[0]
            moved = max(math.hypot(row['x'] - start['x'],
                                   row['y'] - start['y'])
                        for row in odometry) > 0.02
        nonzero = any(abs(row['linear_x']) > 1.0e-6 or
                      abs(row['angular_z']) > 1.0e-6 for row in safe)
        emergency = any(row.get('emergency_stop', False) for row in states)
        passed = (action.get('status') == 6 and odometry and safe and
                  not moved and not nonzero and not emergency)
        return bool(passed), {
            'expected': 'ABORTED without motion', 'action': action,
            'odometry_samples': len(odometry), 'moved_over_0.02m': moved,
            'safe_command_samples': len(safe), 'nonzero_safe_command': nonzero,
            'emergency_stop': emergency}
    settled = next((row['data'] for row in reversed(records)
                    if row['event'] == 'settled_result'), None)
    adapter = [row['data'] for row in records
               if row.get('topic') == '/atlas_ackermann/command_status']
    controls = [row['data'] for row in records
                if row['event'] == 'control_command']
    dynamics = [row['data'] for row in records
                if row['event'] == 'simulator_dynamics']
    path_record = next((row for row in records if row['event'] == 'path'), None)
    path = path_record['data'].get('poses', []) if path_record else []

    def segment_distance(point, start, end):
        dx, dy = end['x'] - start['x'], end['y'] - start['y']
        denominator = dx * dx + dy * dy
        if denominator <= 1.0e-12:
            return math.hypot(point['x'] - start['x'], point['y'] - start['y'])
        fraction = max(0.0, min(1.0,
            ((point['x'] - start['x']) * dx +
             (point['y'] - start['y']) * dy) / denominator))
        return math.hypot(point['x'] - (start['x'] + fraction * dx),
                          point['y'] - (start['y'] + fraction * dy))

    cross_track = [
        min(segment_distance(row, start, end)
            for start, end in zip(path, path[1:]))
        for row in odometry if len(path) > 1]
    world_scenario = 'avoidable' if scenario == 'avoidable' else 'open'
    clearances = [clearance(row['x'], row['y'], ROBOT_RADIUS_M, world_scenario)
                  for row in odometry]
    collision_clamps = sum(bool(row.get('collision_clamp')) for row in dynamics)
    max_acceleration = max((row['acceleration_mps2'] for row in dynamics),
                           default=None)
    min_acceleration = min((row['acceleration_mps2'] for row in dynamics),
                           default=None)
    max_speed = max((abs(row['speed_after_mps']) for row in dynamics), default=None)
    max_dynamic_steering = max((abs(row['steering_after_rad']) for row in dynamics),
                               default=None)
    max_dynamic_steering_rate = max(
        (abs(row['steering_rate_radps']) for row in dynamics), default=None)
    dynamics_feasible = bool(dynamics) and collision_clamps == 0 and \
        max_speed <= MAX_SPEED_MPS + LIMIT_TOLERANCE and \
        max_dynamic_steering <= MAX_STEERING_RAD + LIMIT_TOLERANCE and \
        max_dynamic_steering_rate <= MAX_STEERING_RATE_RADPS + LIMIT_TOLERANCE and \
        max_acceleration <= MAX_ACCELERATION_MPS2 + LIMIT_TOLERANCE and \
        min_acceleration >= -MAX_DECELERATION_MPS2 - LIMIT_TOLERANCE
    passed = (action.get('status') == 4 and settled is not None and
              settled.get('passed') is True and adapter and
              all(item.get('feasible') for item in adapter) and
              not any(item.get('emergency_stop', False) for item in states) and
              dynamics_feasible and bool(clearances) and min(clearances) >= 0.0)
    max_steering = max((abs(item['applied_steering']) for item in adapter),
                       default=None)
    max_rate = max((abs(item['steering_rate_radps']) for item in controls
                    if item.get('steering_rate_radps') is not None), default=None)
    return bool(passed), {
        'action': action, 'settled': settled,
        'adapter_samples': len(adapter), 'all_adapter_commands_feasible': bool(adapter) and all(
            item.get('feasible') for item in adapter),
        'max_applied_steering_rad': max_steering,
        'max_applied_steering_rate_radps': max_rate,
        'max_cross_track_error_m': max(cross_track, default=None),
        'minimum_obstacle_or_boundary_clearance_m': min(clearances, default=None),
        'simulator_collision_clamps': collision_clamps,
        'simulator_dynamics_feasible': dynamics_feasible,
        'max_simulator_speed_mps': max_speed,
        'max_simulator_acceleration_mps2': max_acceleration,
        'min_simulator_acceleration_mps2': min_acceleration,
        'max_simulator_steering_rad': max_dynamic_steering,
        'max_simulator_steering_rate_radps': max_dynamic_steering_rate,
        'control_command_samples': len(controls),
        'safety_states': len(states)}


def stop_group(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('straight', 'curved', 'avoidable', 'blocked'),
                        required=True)
    parser.add_argument('--repetitions', type=int, default=1)
    parser.add_argument('--domain-start', type=int, default=210,
                        help='unused Fast DDS domain range; each trial uses a new ID')
    parser.add_argument('--evidence-root', type=Path, required=True)
    parser.add_argument('--timeout', type=float)
    parser.add_argument('--goal-x', type=float)
    parser.add_argument('--goal-y', type=float)
    parser.add_argument('--goal-yaw', type=float, default=0.0)
    args = parser.parse_args(argv)
    if args.repetitions < 1 or args.domain_start < 0 or \
            args.domain_start + args.repetitions - 1 > 232:
        parser.error('repetitions must be positive and selected ROS domains must be 0..232')

    prefix = Path(get_package_prefix('atlas_nav2_demo')).resolve()
    if not is_development_package_prefix(prefix):
        parser.error(
            'development package must resolve from the first COLCON_PREFIX_PATH '
            'entry (or ATLAS_NAV2_OVERLAY); resolved package prefix: '
            f'{prefix}')
    share = Path(get_package_share_directory('atlas_nav2_demo'))
    map_name = 'blocked_map.yaml' if args.scenario == 'blocked' else (
        'avoidable_map.yaml' if args.scenario == 'avoidable' else 'open_map.yaml')
    simulator_scenario = 'avoidable' if args.scenario == 'avoidable' else (
        'blocked' if args.scenario == 'blocked' else 'open')
    default_goal = ((1.45, -0.05) if args.scenario == 'straight' else (4.45, 1.75))
    goal_x = args.goal_x if args.goal_x is not None else default_goal[0]
    goal_y = args.goal_y if args.goal_y is not None else default_goal[1]
    timeout = args.timeout or (40.0 if args.scenario == 'blocked' else
                               30.0 if args.scenario == 'straight' else 70.0)

    args.evidence_root.mkdir(parents=True, exist_ok=True)
    summary = {'scenario': args.scenario, 'repetitions_requested': args.repetitions,
               'package_prefix': str(prefix), 'trials': []}
    for index in range(args.repetitions):
        domain = args.domain_start + index
        stem = f'{args.scenario}_run{index + 1:02d}_domain{domain}'
        capture_path = args.evidence_root / f'{stem}.jsonl'
        launch_log_path = args.evidence_root / f'{stem}_launch.log'
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(domain)
        command = ['ros2', 'launch', 'atlas_nav2_demo',
                   'nav2_ackermann_controller.launch.py',
                   f'scenario:={simulator_scenario}', f'map:={share / "maps" / map_name}']
        with launch_log_path.open('x', encoding='utf-8') as launch_log:
            launch = subprocess.Popen(command, env=env, stdout=launch_log,
                                      stderr=subprocess.STDOUT,
                                      start_new_session=True)
            try:
                ready_deadline = time.monotonic() + 45.0
                while time.monotonic() < ready_deadline:
                    if launch.poll() is not None:
                        raise RuntimeError(f'Nav2 launch exited {launch.returncode}; see {launch_log_path}')
                    launch_log.flush()
                    if 'Managed nodes are active' in launch_log_path.read_text(
                            encoding='utf-8', errors='replace'):
                        break
                    time.sleep(0.25)
                else:
                    raise TimeoutError(f'Nav2 did not activate within 45 sec; see {launch_log_path}')

                capture_command = [
                    'ros2', 'run', 'atlas_nav2_demo', 'record_ackermann_navigation.py',
                    '--output', str(capture_path), '--goal-x', str(goal_x),
                    '--goal-y', str(goal_y), '--goal-yaw', str(args.goal_yaw),
                    '--timeout', str(timeout), '--settle-timeout', '3', '--enable-gate']
                completed = subprocess.run(capture_command, env=env, check=False)
                if completed.returncode not in (0, 1):
                    raise RuntimeError(f'recorder exited {completed.returncode}; see {capture_path}')
                records = read_records(capture_path)
                passed, metrics = classify(records, args.scenario)
                trial = {'domain': domain, 'capture': str(capture_path),
                         'launch_log': str(launch_log_path),
                         'recorder_exit_code': completed.returncode,
                         'passed': passed, 'metrics': metrics}
                summary['trials'].append(trial)
                if not passed:
                    break
            finally:
                stop_group(launch)

    summary['successes'] = sum(trial['passed'] for trial in summary['trials'])
    summary['failures'] = len(summary['trials']) - summary['successes']
    summary_path = args.evidence_root / f'{args.scenario}_summary.json'
    summary_path.write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(summary, indent=2))
    return 0 if summary['successes'] == args.repetitions else 1


if __name__ == '__main__':
    sys.exit(main())
