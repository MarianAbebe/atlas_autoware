"""Pure constrained bicycle-model primitives for the isolated Ackermann fixture."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AckermannState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    steering: float = 0.0


def require_finite_positive(name, value):
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be finite and positive')


def turning_radius(wheelbase, steering_limit):
    require_finite_positive('wheelbase', wheelbase)
    if not math.isfinite(steering_limit) or not 0.0 < steering_limit < math.pi / 2:
        raise ValueError('steering_limit must be in (0, pi/2)')
    return wheelbase / math.tan(steering_limit)


def clamp(value, limit):
    if not math.isfinite(value) or not math.isfinite(limit) or limit < 0.0:
        raise ValueError('value and limit must be finite; limit nonnegative')
    return max(-limit, min(limit, value))


def step(state, target_speed, target_steering, dt, wheelbase, steering_limit,
         steering_rate_limit, acceleration_limit, deceleration_limit, speed_limit):
    """Advance one bounded bicycle-model step; no in-place rotation is possible."""
    for name, value in (('dt', dt), ('target_speed', target_speed),
                        ('target_steering', target_steering)):
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
    if dt < 0.0:
        raise ValueError('dt must be nonnegative')
    for name, value in (('wheelbase', wheelbase), ('steering_rate_limit', steering_rate_limit),
                        ('acceleration_limit', acceleration_limit),
                        ('deceleration_limit', deceleration_limit), ('speed_limit', speed_limit)):
        require_finite_positive(name, value)
    target_speed = clamp(target_speed, speed_limit)
    target_steering = clamp(target_steering, steering_limit)
    steering_delta = clamp(target_steering - state.steering, steering_rate_limit * dt)
    steering = clamp(state.steering + steering_delta, steering_limit)
    delta_speed = target_speed - state.speed
    speed_delta_limit = (acceleration_limit if delta_speed >= 0.0 else deceleration_limit) * dt
    speed = state.speed + clamp(delta_speed, speed_delta_limit)
    yaw_rate = speed * math.tan(steering) / wheelbase
    yaw = math.atan2(math.sin(state.yaw + yaw_rate * dt), math.cos(state.yaw + yaw_rate * dt))
    return AckermannState(state.x + speed * math.cos(state.yaw) * dt,
                          state.y + speed * math.sin(state.yaw) * dt,
                          yaw, speed, steering), yaw_rate
