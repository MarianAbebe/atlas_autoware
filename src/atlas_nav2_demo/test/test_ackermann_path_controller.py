import json
import math
from pathlib import Path

import pytest

from atlas_nav2_demo.ackermann_kinematics import AckermannState, step
from atlas_nav2_demo.ackermann_path_controller import (
    AckermannPathTracker, Bounds, ControllerConfig, MotionCommand, Pose2D,
    VehicleLimits, analyze_path_geometry, rollout_stop_envelope, rollout_trajectory,
    segment_aabb_distance,
)
from atlas_nav2_demo.ackermann_path_controller import _reachable_value
from atlas_nav2_demo.world import Rect


LIMITS = VehicleLimits()
BOUNDS = Bounds(-1.85, 5.85, -2.85, 2.85)


def arc_path(radius, curvature, angle, points=81):
    result = []
    for index in range(points):
        yaw = math.copysign(angle * index / (points - 1), curvature)
        result.append(Pose2D(math.sin(yaw) / curvature,
                             (1.0 - math.cos(yaw)) / curvature,
                             yaw))
    return result


def tracker(**kwargs):
    return AckermannPathTracker(
        LIMITS, ControllerConfig(prediction_horizon_s=0.4,
                                 prediction_step_s=0.02, **kwargs))


def simulate(path, initial, obstacles=(), config=None, max_steps=2000):
    limits = LIMITS
    controller = AckermannPathTracker(
        limits, config or ControllerConfig(
            lookahead_m=0.45, max_speed_mps=0.40,
            maximum_tracking_error_m=0.50,
            prediction_horizon_s=0.35, prediction_step_s=0.02,
            goal_position_tolerance_m=0.10,
            goal_heading_tolerance_rad=0.15))
    state = initial
    minimum_clearance = math.inf
    max_steering = abs(state.steering)
    max_steering_rate = 0.0
    max_acceleration = 0.0
    max_deceleration = 0.0
    max_cross_track = 0.0
    rejected = 0
    for index in range(max_steps):
        output = controller.compute(state, path, 0.02, obstacles, BOUNDS)
        if not output.accepted:
            rejected += 1
            return {
                'success': False, 'reason': output.reason, 'steps': index,
                'state': state, 'minimum_clearance_m': minimum_clearance,
                'maximum_steering_rad': max_steering,
                'maximum_steering_rate_radps': max_steering_rate,
                'maximum_acceleration_mps2': max_acceleration,
                'maximum_deceleration_mps2': max_deceleration,
                'maximum_cross_track_error_m': max_cross_track,
                'rejected_commands': rejected,
            }
        rollout = output.predicted_trajectory
        minimum_clearance = min(minimum_clearance,
                                rollout.minimum_clearance_m)
        max_steering = max(max_steering, rollout.maximum_steering_rad)
        max_steering_rate = max(max_steering_rate,
                                rollout.maximum_steering_rate_radps)
        max_acceleration = max(max_acceleration,
                               rollout.maximum_acceleration_mps2)
        max_deceleration = max(max_deceleration,
                               rollout.maximum_deceleration_mps2)
        state, _ = step(state, output.command.speed_mps,
                        output.command.steering_rad, 0.02,
                        limits.wheelbase_m, limits.steering_limit_rad,
                        limits.steering_rate_limit_radps,
                        limits.acceleration_limit_mps2,
                        limits.deceleration_limit_mps2,
                        limits.speed_limit_mps)
        cross_track = min(math.hypot(state.x - point.x, state.y - point.y)
                          for point in path)
        max_cross_track = max(max_cross_track, cross_track)
        if output.arrived:
            goal = path[-1]
            return {
                'success': True, 'reason': 'goal_reached', 'steps': index + 1,
                'state': state, 'goal_position_error_m': math.hypot(
                    state.x - goal.x, state.y - goal.y),
                'goal_heading_error_rad': abs(math.atan2(
                    math.sin(state.yaw - goal.yaw),
                    math.cos(state.yaw - goal.yaw))),
                'minimum_clearance_m': minimum_clearance,
                'maximum_steering_rad': max_steering,
                'maximum_steering_rate_radps': max_steering_rate,
                'maximum_acceleration_mps2': max_acceleration,
                'maximum_deceleration_mps2': max_deceleration,
                'maximum_cross_track_error_m': max_cross_track,
                'rejected_commands': rejected,
            }
    return {'success': False, 'reason': 'step_limit', 'steps': max_steps,
            'state': state, 'minimum_clearance_m': minimum_clearance,
            'maximum_steering_rad': max_steering,
            'maximum_steering_rate_radps': max_steering_rate,
            'maximum_acceleration_mps2': max_acceleration,
            'maximum_deceleration_mps2': max_deceleration,
            'maximum_cross_track_error_m': max_cross_track,
            'rejected_commands': rejected}


def test_straight_path_tracking_is_forward_and_feasible():
    path = [Pose2D(x, 0.0, 0.0) for x in (0.0, 0.5, 1.0, 2.0)]
    result = tracker().compute(AckermannState(), path, 0.05, bounds=BOUNDS)
    assert result.accepted
    assert result.command.speed_mps > 0.0
    assert abs(result.requested_curvature_per_m) < 1e-9
    assert result.predicted_trajectory.feasible


def test_constant_radius_turn_respects_steering_limit():
    radius = 3.0
    curvature = 1.0 / radius
    path = arc_path(radius, curvature, 0.8)
    result = tracker().compute(AckermannState(), path, 0.05, bounds=BOUNDS)
    assert result.accepted
    assert result.requested_curvature_per_m == pytest.approx(curvature, abs=2e-3)
    assert abs(result.command.steering_rad) <= LIMITS.steering_limit_rad


def test_tighter_than_minimum_radius_is_rejected_and_decelerates():
    path = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.5, 0.2, 0.5)]
    state = AckermannState(speed=0.25)
    result = tracker().compute(state, path, 0.05)
    assert not result.accepted
    assert result.reason == 'requested_curvature_infeasible'
    assert result.command.speed_mps < state.speed
    assert abs(result.command.steering_rad) <= LIMITS.steering_limit_rad


def test_curved_path_prediction_is_feasible():
    path = arc_path(3.0, 1.0 / 3.0, 0.5)
    state = AckermannState(steering=math.atan(LIMITS.wheelbase_m / 3.0))
    result = tracker().compute(state, path, 0.05, bounds=BOUNDS)
    assert result.accepted
    assert result.predicted_trajectory.feasible
    assert result.predicted_trajectory.maximum_steering_rad <= 0.50


def test_reverse_path_commands_negative_velocity_without_rotation():
    path = [Pose2D(-0.2 * i, 0.0, 0.0) for i in range(6)]
    result = tracker().compute(AckermannState(), path, 0.05, bounds=BOUNDS)
    assert result.accepted
    assert result.command.speed_mps < 0.0
    assert result.predicted_trajectory.states[-1].yaw == pytest.approx(0.0)


def test_reverse_curved_path_uses_body_frame_steering_sign():
    # A path whose body heading turns left while the vehicle travels backward
    # needs the opposite steering curvature from an otherwise identical
    # forward arc.
    path = arc_path(3.0, 1.0 / 3.0, 0.35)
    reverse_path = [Pose2D(-p.x, -p.y, p.yaw) for p in path]
    result = tracker().compute(AckermannState(), reverse_path, 0.05,
                               bounds=BOUNDS)
    assert result.accepted
    assert result.command.speed_mps < 0.0
    assert result.requested_curvature_per_m < 0.0
    assert result.predicted_trajectory.feasible


def test_reversing_cusp_stops_then_selects_reverse_run():
    path = [Pose2D(x, 0.0, 0.0) for x in (0.0, 0.5, 1.0, 1.5, 2.0,
                                            1.8, 1.6, 1.4)]
    report = analyze_path_geometry(path, LIMITS)
    assert report.cusp_indices == (4,)
    approach = tracker().compute(AckermannState(1.8, 0.0, 0.0, 0.2, 0.0),
                                 path, 0.05, bounds=BOUNDS)
    assert approach.accepted
    assert approach.command.speed_mps >= 0.0
    at_cusp = tracker().compute(AckermannState(2.0, 0.0, 0.0, 0.0, 0.0),
                                path, 0.05, bounds=BOUNDS)
    assert at_cusp.accepted
    assert at_cusp.command.speed_mps < 0.0


def test_predicted_collision_rejects_command():
    obstacle = Rect(0.45, 0.55, -0.10, 0.10)
    path = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)]
    result = tracker().compute(AckermannState(speed=0.55), path, 0.05,
                                (obstacle,))
    assert not result.accepted
    assert result.reason == 'latency_braking_envelope_collision'
    assert result.command.speed_mps < 0.55
    assert result.predicted_trajectory is not None
    assert not result.predicted_trajectory.feasible


def test_latency_braking_envelope_preserves_clearance_when_feasible():
    obstacle = Rect(1.2, 1.4, -0.1, 0.1)
    envelope = rollout_stop_envelope(
        AckermannState(speed=0.35), LIMITS, command_latency_s=0.37,
        step_s=0.02, obstacles=(obstacle,), bounds=BOUNDS,
        additional_margin_m=0.05)
    assert envelope.feasible
    assert envelope.states[-1].speed == pytest.approx(0.0)
    assert envelope.minimum_clearance_m > 0.0


def test_complete_feasible_obstacle_detour_reaches_goal():
    radius = 2.40
    curvature = -1.0 / radius
    final_yaw = -math.pi / 2.0
    path = [Pose2D(math.sin(final_yaw * i / 160) / curvature,
                   (1.0 - math.cos(final_yaw * i / 160)) / curvature,
                   final_yaw * i / 160) for i in range(161)]
    # The straight chord to the goal intersects this obstacle; the feasible
    # quarter-circle path passes above it with a full swept-circle margin.
    obstacle = Rect(1.0, 1.2, -1.2, -1.0)
    assert segment_aabb_distance((0.0, 0.0),
                                 (path[-1].x, path[-1].y), obstacle) < 0.01
    detour_config = ControllerConfig(
        lookahead_m=1.5, max_speed_mps=0.20,
        maximum_tracking_error_m=0.50,
        prediction_horizon_s=0.35, prediction_step_s=0.02,
        goal_position_tolerance_m=0.10,
        goal_heading_tolerance_rad=0.15)
    result = simulate(path, AckermannState(), (obstacle,), detour_config)
    assert result['success'], result
    assert result['goal_position_error_m'] <= 0.10
    assert result['goal_heading_error_rad'] <= 0.15
    assert result['minimum_clearance_m'] > 0.0
    assert result['maximum_steering_rad'] <= LIMITS.steering_limit_rad
    assert result['maximum_steering_rate_radps'] <= \
        LIMITS.steering_rate_limit_radps + 1e-9
    assert result['maximum_acceleration_mps2'] <= \
        LIMITS.acceleration_limit_mps2 + 1e-9
    assert result['maximum_deceleration_mps2'] <= \
        LIMITS.deceleration_limit_mps2 + 1e-9
    assert result['rejected_commands'] == 0


def test_zero_speed_has_no_yaw_and_low_speed_is_bounded():
    state = AckermannState(steering=0.4)
    stopped = rollout_trajectory(state, MotionCommand(0.0, 0.4), LIMITS,
                                 0.2, 0.02)
    assert stopped.feasible
    assert all(p.yaw == pytest.approx(state.yaw) for p in stopped.states)
    result = tracker().compute(AckermannState(),
                               [Pose2D(0, 0, 0), Pose2D(1, 0, 0)],
                               0.01)
    assert result.accepted
    assert result.command.speed_mps <= LIMITS.acceleration_limit_mps2 * 0.01


def test_goal_is_not_reported_complete_until_vehicle_is_stopped():
    path = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.5, 0.0, 0.0)]
    state = AckermannState(x=0.45, speed=0.10)
    result = tracker().compute(state, path, 0.05, bounds=BOUNDS)
    assert result.accepted
    assert not result.arrived
    assert result.command.speed_mps < state.speed


@pytest.mark.parametrize('bad_path', [
    [], [Pose2D(0.0, 0.0, 0.0)],
    [Pose2D(0.0, 0.0, 0.0), Pose2D(math.nan, 0.0, 0.0)],
])
def test_malformed_paths_are_rejected(bad_path):
    with pytest.raises(ValueError):
        tracker().compute(AckermannState(), bad_path, 0.05)


def test_in_place_rotation_path_is_safely_rejected():
    path = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, 1.0)]
    result = tracker().compute(AckermannState(), path, 0.05)
    assert not result.accepted
    assert result.reason == 'invalid_or_in_place_path'
    assert result.command.speed_mps == 0.0


def test_lateral_non_ackermann_translation_is_rejected():
    path = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.5, 0.0)]
    result = tracker().compute(AckermannState(), path, 0.05)
    assert not result.accepted
    assert result.reason == 'invalid_or_in_place_path'
    assert analyze_path_geometry(path, LIMITS).non_ackermann_segment_indices == (0,)


def test_steering_rate_and_acceleration_limits_are_applied():
    path = arc_path(3.0, 1.0 / 3.0, 0.8)
    result = tracker().compute(AckermannState(), path, 0.05)
    assert result.accepted
    assert abs(result.command.steering_rad) <= \
        LIMITS.steering_rate_limit_radps * 0.05 + 1e-9
    assert result.command.speed_mps <= LIMITS.acceleration_limit_mps2 * 0.05
    assert result.predicted_trajectory.maximum_steering_rate_radps <= \
        LIMITS.steering_rate_limit_radps + 1e-9
    assert result.predicted_trajectory.maximum_acceleration_mps2 <= \
        LIMITS.acceleration_limit_mps2 + 1e-9


def test_python_reference_matches_cpp_math_golden_vectors():
    # Kept in sync with test_ackermann_controller_math.cpp.
    assert 2.0 * 0.25 / (1.0**2 + 0.25**2) == pytest.approx(0.47058823529411764)
    assert math.atan(1.2 * 0.4) == pytest.approx(math.atan(0.48))
    assert _reachable_value(0.0, 0.2, 0.8, 1.2, 0.1) == pytest.approx(0.08)
    assert _reachable_value(0.08, -0.08, 0.8, 1.2, 0.1) == pytest.approx(0.0)


def test_swept_collision_checks_space_between_prediction_samples():
    small_limits = VehicleLimits(robot_radius_m=0.05, safety_margin_m=0.0,
                                 speed_limit_mps=0.8)
    obstacle = Rect(0.395, 0.405, -0.01, 0.01)
    result = rollout_trajectory(
        AckermannState(speed=0.8), MotionCommand(0.8, 0.0), small_limits,
        horizon_s=1.0, step_s=1.0, obstacles=(obstacle,))
    assert len(result.states) == 2
    assert not result.feasible
    assert result.reason == 'swept_footprint_collision'


@pytest.mark.parametrize('state', [
    AckermannState(x=math.inf), AckermannState(speed=math.nan),
    AckermannState(steering=0.51), AckermannState(speed=0.81),
])
def test_rollout_rejects_nonfinite_or_out_of_limit_initial_state(state):
    with pytest.raises(ValueError):
        rollout_trajectory(state, MotionCommand(0.0, 0.0), LIMITS,
                           horizon_s=0.1, step_s=0.02)


def test_recorded_smac_path_cusp_aware_curvature_diagnostic():
    fixture = (Path(__file__).parent / 'fixtures' /
               'recorded_smac_path.json')
    recorded = json.loads(fixture.read_text(encoding='utf-8'))['poses']
    path = [Pose2D(p['x'], p['y'], p['yaw']) for p in recorded]
    report = analyze_path_geometry(path, LIMITS, window_segment_count=8)
    assert report.cusp_indices == ()
    assert report.direction_runs[0].direction == 1
    assert report.max_segment_curvature_per_m == pytest.approx(0.46491, abs=2e-4)
    assert report.max_window_curvature_per_m == pytest.approx(0.44961, abs=2e-4)
    assert not report.certifiable_from_samples
    assert report.windowed_within_curvature_limit
