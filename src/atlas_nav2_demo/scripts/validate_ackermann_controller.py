#!/usr/bin/env python3
"""Bounded deterministic validation for the isolated Ackermann tracker.

This is a pure local harness: it starts no ROS graph and does not replace the
active Nav2 controller. Evidence is written outside the source repository.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

from atlas_nav2_demo.ackermann_kinematics import AckermannState, step
from atlas_nav2_demo.ackermann_path_controller import (
    AckermannPathTracker, Bounds, ControllerConfig, MotionCommand, Pose2D,
    VehicleLimits, analyze_path_geometry, segment_aabb_distance,
)
from atlas_nav2_demo.world import Rect


DT = 0.02
WORLD = Bounds(-1.85, 5.85, -2.85, 2.85)


def quarter_arc(radius=2.4, count=161):
    curvature = -1.0 / radius
    end_yaw = -math.pi / 2.0
    return [Pose2D(math.sin(end_yaw * i / (count - 1)) / curvature,
                   (1.0 - math.cos(end_yaw * i / (count - 1))) / curvature,
                   end_yaw * i / (count - 1)) for i in range(count)]


def curved_arc(radius=3.0, end_yaw=0.9, count=101):
    curvature = 1.0 / radius
    return [Pose2D(math.sin(end_yaw * i / (count - 1)) / curvature,
                   (1.0 - math.cos(end_yaw * i / (count - 1))) / curvature,
                   end_yaw * i / (count - 1)) for i in range(count)]


def nearest_distance(state, path):
    return min(math.hypot(state.x - pose.x, state.y - pose.y)
               for pose in path)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def run_case(name, path, limits, config, obstacles=(), initial=None,
             expected_rejection=False, max_steps=6000):
    tracker = AckermannPathTracker(limits, config)
    state = initial or AckermannState()
    start = state
    max_steer = abs(state.steering)
    max_steer_rate = max_accel = max_decel = 0.0
    max_requested_steer = max_requested_curvature = 0.0
    max_cross_track = 0.0
    min_clearance = math.inf
    rejection_count = 0
    collision = False
    action = 'RUNNING'
    reason = 'step_limit'
    path_geometry = analyze_path_geometry(path, limits)

    # Initial clearance is included, including map boundaries.
    for step_index in range(max_steps):
        output = tracker.compute(state, path, DT, obstacles, WORLD)
        max_requested_steer = max(max_requested_steer,
                                  abs(output.requested_steering_rad))
        max_requested_curvature = max(max_requested_curvature,
                                       abs(output.requested_curvature_per_m))
        if not output.accepted:
            rejection_count += 1
            action, reason = 'ABORTED_SAFE_STOP', output.reason
            if not expected_rejection:
                break
            # Exercise the returned bounded stop command to standstill, then
            # end this intentionally infeasible/blocked navigation attempt.
            for _ in range(200):
                stop_output = tracker.compute(state, path, DT, obstacles, WORLD)
                previous = state
                state, _ = step(
                    state, stop_output.command.speed_mps,
                    stop_output.command.steering_rad,
                    DT, limits.wheelbase_m, limits.steering_limit_rad,
                    limits.steering_rate_limit_radps,
                    limits.acceleration_limit_mps2,
                    limits.deceleration_limit_mps2, limits.speed_limit_mps)
                distance = segment_aabb_distance(
                    (previous.x, previous.y), (state.x, state.y),
                    obstacles[0]) - limits.robot_radius_m - limits.safety_margin_m if obstacles else math.inf
                boundary_clearance = min(
                    min(previous.x, state.x) - WORLD.x_min,
                    WORLD.x_max - max(previous.x, state.x),
                    min(previous.y, state.y) - WORLD.y_min,
                    WORLD.y_max - max(previous.y, state.y)) - limits.robot_radius_m - limits.safety_margin_m
                min_clearance = min(min_clearance, distance,
                                    boundary_clearance)
                max_steer = max(max_steer, abs(state.steering))
                max_steer_rate = max(max_steer_rate,
                                     abs(state.steering - previous.steering) / DT)
                magnitude_delta = (abs(state.speed) - abs(previous.speed)) / DT
                max_accel = max(max_accel, magnitude_delta, 0.0)
                max_decel = max(max_decel, -magnitude_delta, 0.0)
                if abs(state.speed) < 1e-8:
                    break
            break

        prediction = output.predicted_trajectory
        if prediction is None or not prediction.feasible:
            action, reason = 'ABORTED_SAFE_STOP', 'missing_or_infeasible_prediction'
            break
        min_clearance = min(min_clearance, prediction.minimum_clearance_m)
        max_steer = max(max_steer, prediction.maximum_steering_rad)
        max_steer_rate = max(max_steer_rate,
                             prediction.maximum_steering_rate_radps)
        max_accel = max(max_accel, prediction.maximum_acceleration_mps2)
        max_decel = max(max_decel,
                        prediction.maximum_deceleration_mps2)
        previous = state
        state, _ = step(
            state, output.command.speed_mps, output.command.steering_rad, DT,
            limits.wheelbase_m, limits.steering_limit_rad,
            limits.steering_rate_limit_radps,
            limits.acceleration_limit_mps2, limits.deceleration_limit_mps2,
            limits.speed_limit_mps)
        swept = math.inf
        for obstacle in obstacles:
            swept = min(swept, segment_aabb_distance(
                (previous.x, previous.y), (state.x, state.y), obstacle) -
                limits.robot_radius_m - limits.safety_margin_m)
        boundary = min(
            min(previous.x, state.x) - WORLD.x_min,
            WORLD.x_max - max(previous.x, state.x),
            min(previous.y, state.y) - WORLD.y_min,
            WORLD.y_max - max(previous.y, state.y)) - limits.robot_radius_m - limits.safety_margin_m
        min_clearance = min(min_clearance, swept, boundary)
        if swept < 0.0 or boundary < 0.0:
            collision = True
            action, reason = 'COLLISION', 'actual_swept_footprint_intersection'
            break
        max_cross_track = max(max_cross_track, nearest_distance(state, path))
        if output.arrived:
            action, reason = 'SUCCEEDED', 'goal_reached'
            break

    goal = path[-1]
    position_error = math.hypot(state.x - goal.x, state.y - goal.y)
    heading_error = abs(wrap(state.yaw - goal.yaw))
    if expected_rejection:
        success = action == 'ABORTED_SAFE_STOP' and abs(state.speed) < 1e-8 and not collision
    else:
        success = action == 'SUCCEEDED' and not collision
    return {
        'scenario': name, 'passed': success, 'action_status': action,
        'reason': reason, 'steps': step_index + 1,
        'simulated_duration_s': round((step_index + 1) * DT, 4),
        'start_pose': {'x': start.x, 'y': start.y, 'yaw': start.yaw},
        'final_pose': {'x': state.x, 'y': state.y, 'yaw': state.yaw},
        'final_speed_mps': state.speed,
        'goal_position_error_m': position_error,
        'goal_heading_error_rad': heading_error,
        'maximum_cross_track_error_m': max_cross_track,
        'maximum_steering_rad': max_steer,
        'maximum_requested_steering_rad': max_requested_steer,
        'maximum_requested_curvature_per_m': max_requested_curvature,
        'maximum_steering_rate_radps': max_steer_rate,
        'maximum_acceleration_mps2': max_accel,
        'maximum_deceleration_mps2': max_decel,
        'minimum_obstacle_or_boundary_clearance_m': min_clearance,
        'collision': collision, 'rejected_or_infeasible_commands': rejection_count,
        'path_geometry': {
            'direction_runs': len(path_geometry.direction_runs),
            'cusp_indices': list(path_geometry.cusp_indices),
            'maximum_sampled_curvature_per_m': path_geometry.max_segment_curvature_per_m,
            'maximum_window_curvature_per_m': path_geometry.max_window_curvature_per_m,
            'sampled_within_limit': path_geometry.sampled_within_curvature_limit,
            'windowed_within_limit': path_geometry.windowed_within_curvature_limit,
        },
    }


def cases():
    limits = VehicleLimits()
    nominal = ControllerConfig(lookahead_m=0.60, max_speed_mps=0.35,
                               maximum_tracking_error_m=0.75,
                               prediction_horizon_s=0.35,
                               prediction_step_s=0.02)
    detour = ControllerConfig(lookahead_m=1.50, max_speed_mps=0.20,
                              maximum_tracking_error_m=0.50,
                              prediction_horizon_s=0.35,
                              prediction_step_s=0.02,
                              goal_position_tolerance_m=0.10,
                              goal_heading_tolerance_rad=0.15)
    straight = [Pose2D(x, 0.0, 0.0) for x in (0.0, 0.5, 1.0, 1.5, 2.0)]
    curved = curved_arc(3.0, 0.9, 101)
    detour_path = quarter_arc(2.4)
    obstacle = Rect(1.0, 1.2, -1.2, -1.0)
    too_tight = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.5, 0.2, 0.7),
                 Pose2D(0.9, 0.6, 1.4)]
    blocked_obstacle = Rect(1.4, 1.7, -2.85, 2.85)
    return [
        ("straight", straight, nominal, (), False),
        ("curved", curved, ControllerConfig(
            lookahead_m=0.8, max_speed_mps=0.25,
            maximum_tracking_error_m=0.5, prediction_horizon_s=0.35,
            prediction_step_s=0.02, goal_position_tolerance_m=0.10,
            goal_heading_tolerance_rad=0.15), (), False),
        ("feasible_obstacle_detour", detour_path, detour,
         (obstacle,), False),
        ("infeasible_curvature_rejection", too_tight, nominal, (), True),
        ("blocked_route_safe_stop", straight, nominal,
         (blocked_obstacle,), True),
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path.home() /
                        'validation_evidence/ackermann/' /
                        'controller_prototype/summary.json')
    parser.add_argument('--path-trace', type=Path,
                        help='optional JSONL recorder capture containing a path event')
    args = parser.parse_args()
    limits = VehicleLimits()
    started = time.monotonic()
    results = []
    for name, path, config, obstacles, expect_rejection in cases():
        results.append(run_case(name, path, limits, config, obstacles,
                                expected_rejection=expect_rejection))
    recorded_geometry = None
    if args.path_trace is not None:
        if not args.path_trace.is_file():
            parser.error(f'path trace does not exist: {args.path_trace}')
        with args.path_trace.open(encoding='utf-8') as stream:
            for line in stream:
                item = json.loads(line)
                if item.get('event') == 'path':
                    poses = [Pose2D(p['x'], p['y'], p['yaw'])
                             for p in item['data']['poses']]
                    report = analyze_path_geometry(poses, limits,
                                                   window_segment_count=8)
                    recorded_geometry = {
                        'evidence_file': str(args.path_trace),
                        'pose_count': len(poses),
                        'direction_runs': len(report.direction_runs),
                        'cusp_indices': list(report.cusp_indices),
                        'non_ackermann_segment_indices': list(
                            report.non_ackermann_segment_indices),
                        'maximum_edge_curvature_per_m': report.max_segment_curvature_per_m,
                        'maximum_8_edge_window_curvature_per_m': report.max_window_curvature_per_m,
                        'maximum_curvature_limit_per_m': limits.max_curvature_per_m,
                        'sampled_within_limit': report.sampled_within_curvature_limit,
                        'windowed_within_limit': report.windowed_within_curvature_limit,
                        'strictly_certifiable_from_samples': report.certifiable_from_samples,
                        'interpretation': 'edge samples exceed configured limit slightly; 8-edge estimate is within, so samples alone do not certify continuous primitive feasibility',
                    }
                    break
    summary = {
        'schema': 'atlas_ackermann_controller_prototype_v1',
        'created_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'wall_duration_s': round(time.monotonic() - started, 3),
        'execution': 'deterministic standalone Python; no ROS graph/Nav2 plugin',
        'simulation_assumptions': asdict(limits),
        'fixed_step_s': DT,
        'recorded_smac_path_diagnostic': recorded_geometry,
        'results': results,
        'passed': sum(result['passed'] for result in results),
        'failed': sum(not result['passed'] for result in results),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2) + '\n',
                           encoding='utf-8')
    print(json.dumps({'output': str(args.output), 'passed': summary['passed'],
                      'failed': summary['failed'],
                      'results': [{k: r[k] for k in
                                   ('scenario', 'passed', 'action_status',
                                    'reason', 'goal_position_error_m',
                                    'minimum_obstacle_or_boundary_clearance_m',
                                    'maximum_steering_rad',
                                    'rejected_or_infeasible_commands')}
                                  for r in results]}, indent=2))
    return 0 if summary['failed'] == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
