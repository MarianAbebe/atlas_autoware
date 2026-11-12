"""Isolated Ackermann path tracker, bicycle rollout, and swept-circle checker.

This module has no ROS dependencies and is not a Nav2 controller plugin. It
uses the same constrained bicycle-model step as the standalone simulator.
"""
from dataclasses import dataclass, replace
import math
from typing import Iterable, Sequence

from .ackermann_kinematics import AckermannState, step


_EPS = 1.0e-9


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VehicleLimits:
    wheelbase_m: float = 1.20
    steering_limit_rad: float = 0.50
    # Simulation assumption from ackermann_simulation.yaml, not a measured
    # Atlas actuator rate.
    steering_rate_limit_radps: float = 0.60
    acceleration_limit_mps2: float = 0.80
    deceleration_limit_mps2: float = 1.20
    speed_limit_mps: float = 0.80
    robot_radius_m: float = 0.32
    safety_margin_m: float = 0.05

    def __post_init__(self):
        for name in (
                'wheelbase_m', 'steering_rate_limit_radps',
                'acceleration_limit_mps2', 'deceleration_limit_mps2',
                'speed_limit_mps', 'robot_radius_m'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if (not math.isfinite(self.steering_limit_rad) or
                not 0.0 < self.steering_limit_rad < math.pi / 2.0):
            raise ValueError('steering_limit_rad must be in (0, pi/2)')
        if not math.isfinite(self.safety_margin_m) or self.safety_margin_m < 0.0:
            raise ValueError('safety_margin_m must be finite and nonnegative')

    @property
    def max_curvature_per_m(self) -> float:
        return math.tan(self.steering_limit_rad) / self.wheelbase_m

    @property
    def minimum_turning_radius_m(self) -> float:
        return 1.0 / self.max_curvature_per_m


@dataclass(frozen=True)
class Bounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self):
        values = (self.x_min, self.x_max, self.y_min, self.y_max)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('bounds must be finite')
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError('bounds minima must be less than maxima')


@dataclass(frozen=True)
class MotionCommand:
    speed_mps: float
    steering_rad: float


@dataclass(frozen=True)
class PathSegment:
    start_index: int
    end_index: int
    direction: int
    length_m: float
    heading_change_rad: float


@dataclass(frozen=True)
class DirectionRun:
    order: int
    start_index: int
    end_index: int
    direction: int
    segments: tuple[PathSegment, ...]
    length_m: float


@dataclass(frozen=True)
class PathGeometryReport:
    direction_runs: tuple[DirectionRun, ...]
    cusp_indices: tuple[int, ...]
    skipped_zero_length_segments: tuple[int, ...]
    non_ackermann_segment_indices: tuple[int, ...]
    max_segment_curvature_per_m: float
    max_window_curvature_per_m: float
    window_segment_count: int
    minimum_radius_from_window_m: float
    sampled_within_curvature_limit: bool
    windowed_within_curvature_limit: bool
    certifiable_from_samples: bool


@dataclass(frozen=True)
class TrajectoryResult:
    feasible: bool
    reason: str
    states: tuple[AckermannState, ...]
    minimum_clearance_m: float
    maximum_steering_rad: float
    maximum_steering_rate_radps: float
    maximum_acceleration_mps2: float
    maximum_deceleration_mps2: float


@dataclass(frozen=True)
class ControllerConfig:
    lookahead_m: float = 0.60
    max_speed_mps: float = 0.55
    maximum_tracking_error_m: float = 0.75
    prediction_horizon_s: float = 0.80
    prediction_step_s: float = 0.05
    goal_position_tolerance_m: float = 0.10
    goal_heading_tolerance_rad: float = 0.12
    cusp_stop_tolerance_m: float = 0.04
    command_latency_s: float = 0.37
    stopping_buffer_m: float = 0.05

    def __post_init__(self):
        for name in ('lookahead_m', 'max_speed_mps',
                     'maximum_tracking_error_m', 'prediction_horizon_s',
                     'prediction_step_s', 'goal_position_tolerance_m',
                     'goal_heading_tolerance_rad', 'cusp_stop_tolerance_m',
                     'command_latency_s'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if (not math.isfinite(self.stopping_buffer_m) or
                self.stopping_buffer_m < 0.0):
            raise ValueError('stopping_buffer_m must be finite and nonnegative')


@dataclass(frozen=True)
class ControllerOutput:
    accepted: bool
    arrived: bool
    reason: str
    command: MotionCommand
    requested_curvature_per_m: float
    requested_steering_rad: float
    target_index: int
    target_pose: Pose2D
    candidate_trajectory: TrajectoryResult | None
    predicted_trajectory: TrajectoryResult | None


def _finite_pose(pose: Pose2D) -> bool:
    return all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw))


def _path_segments(path: Sequence[Pose2D], zero_length_tolerance_m: float,
                   heading_tolerance_rad: float):
    segments = []
    skipped = []
    unsupported = []
    for index, (a, b) in enumerate(zip(path, path[1:])):
        dx, dy = b.x - a.x, b.y - a.y
        length = math.hypot(dx, dy)
        dyaw = wrap_angle(b.yaw - a.yaw)
        if length <= zero_length_tolerance_m:
            skipped.append(index)
            if abs(dyaw) > heading_tolerance_rad:
                unsupported.append(index)
            continue
        middle_yaw = wrap_angle(a.yaw + 0.5 * dyaw)
        projection = dx * math.cos(middle_yaw) + dy * math.sin(middle_yaw)
        if abs(projection) <= zero_length_tolerance_m:
            direction = 0
            unsupported.append(index)
        else:
            direction = 1 if projection > 0.0 else -1
        segments.append(PathSegment(index, index + 1, direction, length, dyaw))
    return segments, skipped, unsupported


def _make_runs(segments: Sequence[PathSegment]) -> tuple[DirectionRun, ...]:
    runs = []
    current = []
    current_direction = None
    for segment in segments:
        if segment.direction == 0:
            if current:
                runs.append(current)
                current = []
                current_direction = None
            continue
        if current and segment.direction != current_direction:
            runs.append(current)
            current = []
        current_direction = segment.direction
        current.append(segment)
    if current:
        runs.append(current)
    return tuple(DirectionRun(
        order=i,
        start_index=run[0].start_index,
        end_index=run[-1].end_index,
        direction=run[0].direction,
        segments=tuple(run),
        length_m=sum(seg.length_m for seg in run))
        for i, run in enumerate(runs))


def analyze_path_geometry(path: Sequence[Pose2D], limits: VehicleLimits,
                          window_segment_count: int = 8,
                          zero_length_tolerance_m: float = 1.0e-5,
                          heading_tolerance_rad: float = 1.0e-3,
                          curvature_tolerance_per_m: float = 1.0e-6
                          ) -> PathGeometryReport:
    """Estimate sampled curvature without differentiating across direction cusps.

    Single-edge estimates expose lattice/interpolation noise. The fixed-window
    estimate regresses wrapped heading change over accumulated signed path
    length within one direction run. Neither estimate substitutes for the
    planner's continuous primitive model.
    """
    if len(path) < 2 or not all(_finite_pose(p) for p in path):
        raise ValueError('path must contain at least two finite poses')
    if window_segment_count < 2:
        raise ValueError('window_segment_count must be at least two')
    segments, skipped, unsupported = _path_segments(
        path, zero_length_tolerance_m, heading_tolerance_rad)
    if not segments:
        raise ValueError('path contains no nonzero-length segments')
    runs = _make_runs(segments)
    cusps = tuple(run.start_index for run in runs[1:])
    edge_curvatures = [
        abs(seg.heading_change_rad / (seg.direction * seg.length_m))
        for seg in segments if seg.direction != 0]
    max_edge = max(edge_curvatures, default=0.0)
    window_curvatures = []
    for run in runs:
        count = min(window_segment_count, len(run.segments))
        if count < 2:
            continue
        for start in range(len(run.segments) - count + 1):
            window = run.segments[start:start + count]
            yaw_change = sum(seg.heading_change_rad for seg in window)
            signed_distance = window[0].direction * sum(
                seg.length_m for seg in window)
            if abs(signed_distance) > zero_length_tolerance_m:
                window_curvatures.append(abs(yaw_change / signed_distance))
    max_window = max(window_curvatures, default=0.0)
    limit = limits.max_curvature_per_m
    sampled_ok = (max_edge <= limit + curvature_tolerance_per_m and
                  not unsupported)
    window_ok = (max_window <= limit + curvature_tolerance_per_m and
                 not unsupported)
    # If local samples exceed the limit but the larger same-direction window
    # does not, quantization/interpolation prevents strict certification.
    certifiable = sampled_ok
    return PathGeometryReport(
        direction_runs=runs,
        cusp_indices=cusps,
        skipped_zero_length_segments=tuple(skipped),
        non_ackermann_segment_indices=tuple(unsupported),
        max_segment_curvature_per_m=max_edge,
        max_window_curvature_per_m=max_window,
        window_segment_count=window_segment_count,
        minimum_radius_from_window_m=(1.0 / max_window if max_window > _EPS
                                      else math.inf),
        sampled_within_curvature_limit=sampled_ok,
        windowed_within_curvature_limit=window_ok,
        certifiable_from_samples=certifiable)


def _segment_intersects_aabb(a, b, rect) -> bool:
    t0, t1 = 0.0, 1.0
    dx, dy = b[0] - a[0], b[1] - a[1]
    for p, q in ((-dx, a[0] - rect.xmin), (dx, rect.xmax - a[0]),
                 (-dy, a[1] - rect.ymin), (dy, rect.ymax - a[1])):
        if abs(p) <= _EPS:
            if q < 0.0:
                return False
            continue
        ratio = q / p
        if p < 0.0:
            if ratio > t1:
                return False
            t0 = max(t0, ratio)
        else:
            if ratio < t0:
                return False
            t1 = min(t1, ratio)
    return t0 <= t1


def _point_aabb_distance(point, rect) -> float:
    dx = max(rect.xmin - point[0], 0.0, point[0] - rect.xmax)
    dy = max(rect.ymin - point[1], 0.0, point[1] - rect.ymax)
    return math.hypot(dx, dy)


def _point_segment_distance(point, a, b) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    denom = dx * dx + dy * dy
    if denom <= _EPS:
        return math.hypot(point[0] - a[0], point[1] - a[1])
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx +
                          (point[1] - a[1]) * dy) / denom))
    return math.hypot(point[0] - (a[0] + t * dx),
                      point[1] - (a[1] + t * dy))


def segment_aabb_distance(a, b, rect) -> float:
    """Exact minimum distance between a line segment and an axis-aligned rect."""
    if _segment_intersects_aabb(a, b, rect):
        return 0.0
    corners = ((rect.xmin, rect.ymin), (rect.xmin, rect.ymax),
               (rect.xmax, rect.ymin), (rect.xmax, rect.ymax))
    return min(
        _point_aabb_distance(a, rect), _point_aabb_distance(b, rect),
        *(_point_segment_distance(c, a, b) for c in corners))


def _swept_clearance(a, b, limits: VehicleLimits, obstacles: Iterable,
                     bounds: Bounds | None) -> float:
    radius = limits.robot_radius_m + limits.safety_margin_m
    values = [math.inf]
    for obstacle in obstacles:
        values.append(segment_aabb_distance(a, b, obstacle) - radius)
    if bounds is not None:
        values.extend((min(a[0], b[0]) - bounds.x_min - radius,
                       bounds.x_max - max(a[0], b[0]) - radius,
                       min(a[1], b[1]) - bounds.y_min - radius,
                       bounds.y_max - max(a[1], b[1]) - radius))
    return min(values)


def _validate_command(command: MotionCommand, limits: VehicleLimits):
    if not all(math.isfinite(v) for v in
               (command.speed_mps, command.steering_rad)):
        raise ValueError('command must be finite')
    if abs(command.speed_mps) > limits.speed_limit_mps + _EPS:
        raise ValueError('command exceeds speed limit')
    if abs(command.steering_rad) > limits.steering_limit_rad + _EPS:
        raise ValueError('command exceeds steering limit')


def rollout_trajectory(initial: AckermannState, command: MotionCommand,
                       limits: VehicleLimits, horizon_s: float = 1.0,
                       step_s: float = 0.05, obstacles: Iterable = (),
                       bounds: Bounds | None = None) -> TrajectoryResult:
    """Predict one constant target command with rate/acceleration-limited steps.

    Collision checks use the swept circular footprint over every chord between
    adjacent simulator steps, so a narrow obstacle cannot hide between samples.
    """
    if not all(math.isfinite(value) for value in
               (initial.x, initial.y, initial.yaw, initial.speed,
                initial.steering)):
        raise ValueError('initial state must contain only finite values')
    if (abs(initial.speed) > limits.speed_limit_mps + _EPS or
            abs(initial.steering) > limits.steering_limit_rad + _EPS):
        raise ValueError('initial state exceeds configured vehicle limits')
    _validate_command(command, limits)
    if not all(math.isfinite(v) and v > 0.0 for v in (horizon_s, step_s)):
        raise ValueError('horizon_s and step_s must be finite and positive')
    obstacles = tuple(obstacles)
    states = [initial]
    first = _swept_clearance((initial.x, initial.y), (initial.x, initial.y),
                             limits, obstacles, bounds)
    minimum_clearance = first
    if first < 0.0:
        return TrajectoryResult(False, 'initial_footprint_collision', tuple(states),
                                first, abs(initial.steering), 0.0, 0.0, 0.0)
    current = initial
    maximum_steering = abs(initial.steering)
    maximum_steering_rate = 0.0
    maximum_acceleration = 0.0
    maximum_deceleration = 0.0
    elapsed = 0.0
    reason = 'clear'
    while elapsed < horizon_s - _EPS:
        dt = min(step_s, horizon_s - elapsed)
        previous = current
        reachable_speed = _reachable_value(
            current.speed, command.speed_mps,
            limits.acceleration_limit_mps2,
            limits.deceleration_limit_mps2, dt)
        current, _ = step(
            current, reachable_speed, command.steering_rad, dt,
            limits.wheelbase_m, limits.steering_limit_rad,
            limits.steering_rate_limit_radps,
            limits.acceleration_limit_mps2,
            limits.deceleration_limit_mps2,
            limits.speed_limit_mps)
        clearance = _swept_clearance(
            (previous.x, previous.y), (current.x, current.y), limits,
            obstacles, bounds)
        minimum_clearance = min(minimum_clearance, clearance)
        maximum_steering = max(maximum_steering, abs(current.steering))
        maximum_steering_rate = max(
            maximum_steering_rate,
            abs(current.steering - previous.steering) / dt)
        magnitude_delta = (abs(current.speed) - abs(previous.speed)) / dt
        if magnitude_delta >= 0.0:
            maximum_acceleration = max(maximum_acceleration, magnitude_delta)
        else:
            maximum_deceleration = max(maximum_deceleration, -magnitude_delta)
        states.append(current)
        elapsed += dt
        if clearance < 0.0:
            reason = 'swept_footprint_collision'
            return TrajectoryResult(False, reason, tuple(states),
                                    minimum_clearance, maximum_steering,
                                    maximum_steering_rate, maximum_acceleration,
                                    maximum_deceleration)
    return TrajectoryResult(True, reason, tuple(states), minimum_clearance,
                            maximum_steering, maximum_steering_rate,
                            maximum_acceleration, maximum_deceleration)


def rollout_stop_envelope(initial: AckermannState, limits: VehicleLimits,
                          command_latency_s: float, step_s: float,
                          obstacles: Iterable = (), bounds: Bounds | None = None,
                          additional_margin_m: float = 0.0
                          ) -> TrajectoryResult:
    """Hold the last command through latency, then brake with bounded decel.

    Latency and the extra clearance buffer are explicit fixture assumptions.
    The rectangle/costmap-independent geometry is identical to the simulator's
    discrete-step center sweep.
    """
    if (not all(math.isfinite(v) for v in
                (initial.x, initial.y, initial.yaw, initial.speed,
                 initial.steering, command_latency_s, step_s,
                 additional_margin_m)) or
            command_latency_s < 0.0 or step_s <= 0.0 or
            additional_margin_m < 0.0):
        raise ValueError('invalid stop-envelope state or limits')
    effective = replace(
        limits, safety_margin_m=limits.safety_margin_m + additional_margin_m)
    obstacles = tuple(obstacles)
    current = initial
    states = [current]
    minimum_clearance = _swept_clearance(
        (current.x, current.y), (current.x, current.y), effective,
        obstacles, bounds)
    if minimum_clearance < 0.0:
        return TrajectoryResult(False, 'initial_footprint_collision',
                                tuple(states), minimum_clearance,
                                abs(current.steering), 0.0, 0.0, 0.0)
    max_steering = abs(current.steering)
    max_rate = max_accel = max_decel = 0.0
    elapsed = 0.0
    braking = False
    max_time = command_latency_s + abs(initial.speed) / limits.deceleration_limit_mps2 + 2 * step_s
    while elapsed < max_time - _EPS:
        dt = min(step_s, max_time - elapsed)
        if not braking and elapsed >= command_latency_s - _EPS:
            braking = True
        target_speed = 0.0 if braking else initial.speed
        previous = current
        current, _ = step(
            current, target_speed, initial.steering, dt,
            limits.wheelbase_m, limits.steering_limit_rad,
            limits.steering_rate_limit_radps,
            limits.acceleration_limit_mps2,
            limits.deceleration_limit_mps2, limits.speed_limit_mps)
        clearance = _swept_clearance(
            (previous.x, previous.y), (current.x, current.y), effective,
            obstacles, bounds)
        minimum_clearance = min(minimum_clearance, clearance)
        max_steering = max(max_steering, abs(current.steering))
        max_rate = max(max_rate, abs(current.steering - previous.steering) / dt)
        delta = (abs(current.speed) - abs(previous.speed)) / dt
        max_accel = max(max_accel, delta, 0.0)
        max_decel = max(max_decel, -delta, 0.0)
        states.append(current)
        elapsed += dt
        if clearance < 0.0:
            return TrajectoryResult(False, 'latency_braking_envelope_collision',
                                    tuple(states), minimum_clearance,
                                    max_steering, max_rate, max_accel, max_decel)
        if braking and abs(current.speed) <= _EPS:
            break
    return TrajectoryResult(True, 'clear', tuple(states), minimum_clearance,
                            max_steering, max_rate, max_accel, max_decel)


def _reachable_value(current: float, target: float, positive_rate: float,
                     negative_rate: float, dt: float) -> float:
    if current * target < 0.0:
        # A direction reversal must decelerate to zero before accelerating
        # backwards; it cannot cross through zero in one controller update.
        magnitude = max(0.0, abs(current) - negative_rate * dt)
        return math.copysign(magnitude, current) if magnitude else 0.0
    rate = positive_rate if abs(target) > abs(current) else negative_rate
    return current + max(-rate * dt, min(rate * dt, target - current))


def _interpolate_pose(a: Pose2D, b: Pose2D, fraction: float) -> Pose2D:
    dyaw = wrap_angle(b.yaw - a.yaw)
    return Pose2D(a.x + fraction * (b.x - a.x),
                  a.y + fraction * (b.y - a.y),
                  wrap_angle(a.yaw + fraction * dyaw))


def _project_on_run(pose: Pose2D, run: DirectionRun, path: Sequence[Pose2D],
                    speed: float):
    cumulative = 0.0
    best = None
    for segment in run.segments:
        a, b = path[segment.start_index], path[segment.end_index]
        dx, dy = b.x - a.x, b.y - a.y
        length2 = dx * dx + dy * dy
        t = max(0.0, min(1.0,
                         ((pose.x - a.x) * dx + (pose.y - a.y) * dy) / length2))
        projected = _interpolate_pose(a, b, t)
        distance = math.hypot(pose.x - projected.x, pose.y - projected.y)
        heading_error = abs(wrap_angle(pose.yaw - projected.yaw))
        direction_penalty = (0.0 if abs(speed) < 0.05 or
                             (1 if speed > 0.0 else -1) == run.direction
                             else 0.25)
        score = distance + 0.15 * heading_error + direction_penalty
        along = cumulative + t * segment.length_m
        candidate = (score, distance, heading_error, along,
                     segment.start_index, projected)
        if best is None or candidate[0] < best[0] - 1.0e-9:
            best = candidate
        cumulative += segment.length_m
    return best


def _point_at_distance(run: DirectionRun, path: Sequence[Pose2D], distance: float):
    remaining = max(0.0, min(run.length_m, distance))
    for segment in run.segments:
        if remaining <= segment.length_m or segment is run.segments[-1]:
            fraction = min(1.0, remaining / segment.length_m)
            return (_interpolate_pose(path[segment.start_index],
                                      path[segment.end_index], fraction),
                    segment.start_index)
        remaining -= segment.length_m
    return path[run.end_index], run.end_index - 1


class AckermannPathTracker:
    """Stateless-step pure-pursuit tracker with explicit Ackermann rejection."""

    def __init__(self, limits: VehicleLimits,
                 config: ControllerConfig = ControllerConfig()):
        self.limits = limits
        self.config = config
        if config.max_speed_mps > limits.speed_limit_mps:
            raise ValueError('controller max speed exceeds vehicle speed limit')

    def _safe_stop_command(self, state: AckermannState, dt: float) -> MotionCommand:
        speed = _reachable_value(
            state.speed, 0.0, self.limits.acceleration_limit_mps2,
            self.limits.deceleration_limit_mps2, dt)
        steering = _reachable_value(
            state.steering, state.steering,
            self.limits.steering_rate_limit_radps,
            self.limits.steering_rate_limit_radps, dt)
        return MotionCommand(speed, steering)

    def compute(self, state: AckermannState, path: Sequence[Pose2D], dt_s: float,
                obstacles: Iterable = (), bounds: Bounds | None = None
                ) -> ControllerOutput:
        if (not all(math.isfinite(v) for v in
                    (state.x, state.y, state.yaw, state.speed, state.steering)) or
                not math.isfinite(dt_s) or dt_s <= 0.0):
            raise ValueError('state and dt must be finite; dt must be positive')
        if len(path) < 2 or not all(_finite_pose(p) for p in path):
            raise ValueError('path must contain at least two finite poses')
        segments, _, unsupported = _path_segments(path, 1.0e-5, 1.0e-3)
        runs = _make_runs(segments)
        if not runs or unsupported:
            return self._rejected(state, dt_s, 'invalid_or_in_place_path',
                                  0.0, -1, path[0], obstacles, bounds)

        stop_envelope = rollout_stop_envelope(
            state, self.limits, self.config.command_latency_s,
            min(self.config.prediction_step_s, dt_s), obstacles, bounds,
            self.config.stopping_buffer_m)
        if not stop_envelope.feasible:
            stopped = self._rejected(
                state, dt_s, 'latency_braking_envelope_collision',
                0.0, -1, state_to_pose(state), obstacles, bounds)
            return ControllerOutput(
                stopped.accepted, stopped.arrived, stopped.reason,
                stopped.command, stopped.requested_curvature_per_m,
                stopped.requested_steering_rad, stopped.target_index,
                stopped.target_pose, stop_envelope, stop_envelope)

        choices = []
        for run in runs:
            projection = _project_on_run(state_to_pose(state), run, path,
                                         state.speed)
            if projection is not None:
                choices.append((projection[0], run.order, run, projection))
        if not choices:
            return self._rejected(state, dt_s, 'no_trackable_path_segment',
                                  0.0, -1, path[0], obstacles, bounds)
        choices.sort(key=lambda item: (round(item[0], 9), item[1]))
        # A moving vehicle continues its current direction at a cusp. At rest,
        # equal-distance cusp candidates resolve to the later run so reversing
        # can begin only after the preceding run has stopped.
        if abs(state.speed) >= 0.05:
            matching = [c for c in choices
                        if c[2].direction == (1 if state.speed > 0.0 else -1)]
            selected = matching[0] if matching else choices[0]
        else:
            minimum_score = choices[0][0]
            tied = [c for c in choices if c[0] <= minimum_score + 1.0e-7]
            selected = max(tied, key=lambda c: c[1])
            if selected[2].start_index == 0:
                first_run = next((c for c in choices if c[2].start_index == 0), None)
                if first_run is not None:
                    selected = first_run
        _, _, run, projection = selected
        _, distance_error, _, progress, _, projected_pose = projection
        if distance_error > self.config.maximum_tracking_error_m:
            return self._rejected(state, dt_s, 'path_tracking_error',
                                  0.0, run.start_index, projected_pose,
                                  obstacles, bounds)

        goal = path[-1]
        goal_distance = math.hypot(state.x - goal.x, state.y - goal.y)
        goal_heading_error = abs(wrap_angle(state.yaw - goal.yaw))
        if (goal_distance <= self.config.goal_position_tolerance_m and
                goal_heading_error <= self.config.goal_heading_tolerance_rad):
            # Do not report completion while the vehicle is still moving.
            # First command a bounded deceleration; the next update can mark
            # arrival after standstill is reached inside the goal tolerances.
            if abs(state.speed) > 0.02:
                braking_speed = _reachable_value(
                    state.speed, 0.0, self.limits.acceleration_limit_mps2,
                    self.limits.deceleration_limit_mps2, dt_s)
                braking = MotionCommand(braking_speed, state.steering)
                return self._accepted_command(
                    state, dt_s, braking, 0.0, len(path) - 1, goal,
                    obstacles, bounds, prediction_command=MotionCommand(
                        0.0, state.steering))
            output = self._accepted_command(
                state, dt_s, MotionCommand(0.0, state.steering), 0.0,
                len(path) - 1, goal, obstacles, bounds, arrived=True,
                prediction_command=MotionCommand(0.0, state.steering))
            return output

        target_distance = min(run.length_m, progress + self.config.lookahead_m)
        target, target_index = _point_at_distance(run, path, target_distance)
        dx, dy = target.x - state.x, target.y - state.y
        local_x = math.cos(state.yaw) * dx + math.sin(state.yaw) * dy
        local_y = -math.sin(state.yaw) * dx + math.cos(state.yaw) * dy
        distance2 = local_x * local_x + local_y * local_y
        if distance2 <= 1.0e-8:
            return self._rejected(state, dt_s, 'degenerate_lookahead',
                                  0.0, target_index, target, obstacles, bounds)
        # local_y is in the body frame. With a reverse target behind the
        # vehicle, its sign already yields the steering curvature required
        # by yaw_rate = signed_velocity * tan(steering) / wheelbase.
        curvature = 2.0 * local_y / distance2
        steering = math.atan(self.limits.wheelbase_m * curvature)
        if (abs(curvature) > self.limits.max_curvature_per_m + 1.0e-9 or
                abs(steering) > self.limits.steering_limit_rad + 1.0e-9):
            return self._rejected(state, dt_s, 'requested_curvature_infeasible',
                                  curvature, target_index, target,
                                  obstacles, bounds)

        desired_speed = run.direction * self.config.max_speed_mps
        distance_to_run_end = max(0.0, run.length_m - progress)
        has_cusp_after = run.order + 1 < len(runs)
        if has_cusp_after:
            # Stop at the cusp before the next run can select the opposite sign.
            cap = math.sqrt(2.0 * self.limits.deceleration_limit_mps2 *
                            max(0.0, distance_to_run_end -
                                self.config.cusp_stop_tolerance_m))
            desired_speed = math.copysign(
                min(abs(desired_speed), cap), run.direction)

        bounded_speed = _reachable_value(
            state.speed, desired_speed,
            self.limits.acceleration_limit_mps2,
            self.limits.deceleration_limit_mps2, dt_s)
        bounded_steering = _reachable_value(
            state.steering, steering,
            self.limits.steering_rate_limit_radps,
            self.limits.steering_rate_limit_radps, dt_s)
        command = MotionCommand(bounded_speed, bounded_steering)
        prediction_command = MotionCommand(desired_speed, steering)
        return self._accepted_command(
            state, dt_s, command, curvature, target_index, target,
            obstacles, bounds, prediction_command=prediction_command)

    def _rejected(self, state, dt_s, reason, curvature, target_index, target,
                  obstacles, bounds):
        command = self._safe_stop_command(state, dt_s)
        stop_target = MotionCommand(0.0, state.steering)
        predicted = rollout_trajectory(
            state, stop_target, self.limits,
            self.config.prediction_horizon_s,
            self.config.prediction_step_s, obstacles, bounds)
        return ControllerOutput(
            False, False, reason, command, curvature,
            math.atan(self.limits.wheelbase_m * curvature), target_index,
            target, None, predicted)

    def _accepted_command(self, state, dt_s, command, curvature,
                          target_index, target, obstacles, bounds,
                          arrived=False, prediction_command=None):
        candidate = rollout_trajectory(
            state, prediction_command or command, self.limits,
            self.config.prediction_horizon_s,
            self.config.prediction_step_s, obstacles, bounds)
        if not candidate.feasible:
            stop = self._safe_stop_command(state, dt_s)
            stop_target = MotionCommand(0.0, state.steering)
            stop_prediction = rollout_trajectory(
                state, stop_target, self.limits,
                self.config.prediction_horizon_s,
                self.config.prediction_step_s, obstacles, bounds)
            return ControllerOutput(
                False, False, candidate.reason, stop, curvature,
                math.atan(self.limits.wheelbase_m * curvature), target_index,
                target, candidate, stop_prediction)
        return ControllerOutput(True, arrived, 'goal_reached' if arrived else 'tracking',
                                command, curvature,
                                math.atan(self.limits.wheelbase_m * curvature),
                                target_index, target, candidate, candidate)


def state_to_pose(state: AckermannState) -> Pose2D:
    return Pose2D(state.x, state.y, state.yaw)
