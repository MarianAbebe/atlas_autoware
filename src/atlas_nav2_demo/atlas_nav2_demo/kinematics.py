"""Pure ideal skid-steer kinematics used by the mixer, simulator, and tests."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WheelSpeeds:
    front_left: float
    rear_left: float
    front_right: float
    rear_right: float


def mix_twist(linear: float, angular: float, track_width: float,
              wheel_radius: float) -> WheelSpeeds:
    """Convert planar body velocity to four ideal wheel angular velocities."""
    if not math.isfinite(track_width) or track_width <= 0.0:
        raise ValueError('track_width must be finite and positive')
    if not math.isfinite(wheel_radius) or wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be finite and positive')
    if not math.isfinite(linear) or not math.isfinite(angular):
        raise ValueError('command velocities must be finite')
    left = (linear - angular * track_width / 2.0) / wheel_radius
    right = (linear + angular * track_width / 2.0) / wheel_radius
    return WheelSpeeds(left, left, right, right)


def wheels_to_twist(left: float, right: float, track_width: float,
                    wheel_radius: float) -> tuple[float, float]:
    """Return ideal body linear/angular velocity from left/right wheel rates."""
    if track_width <= 0.0 or wheel_radius <= 0.0:
        raise ValueError('geometry must be positive')
    linear = wheel_radius * (right + left) / 2.0
    angular = wheel_radius * (right - left) / track_width
    return linear, angular
