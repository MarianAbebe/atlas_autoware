"""Pure Ackermann command conversion used by the ROS adapter."""
import math


def convert_command(linear, angular, wheelbase, steering_limit, speed_limit,
                    minimum_conversion_speed):
    """Return bounded velocity, requested curvature/steering, and feasibility.

    Infeasible commands return zero velocity and zero applied steering. The
    requested values are retained for diagnostics; no steering clipping occurs.
    """
    if not all(math.isfinite(value) for value in
               (linear, angular, wheelbase, steering_limit, speed_limit,
                minimum_conversion_speed)):
        return 0.0, 0.0, 0.0, False
    if min(wheelbase, steering_limit, speed_limit,
           minimum_conversion_speed) <= 0.0:
        raise ValueError('Ackermann limits must be positive')

    velocity = max(-speed_limit, min(speed_limit, linear))
    if abs(velocity) < minimum_conversion_speed:
        # Ackermann vehicles cannot satisfy a pure angular command at (near)
        # zero speed. Treat it as infeasible and stop instead of hiding it.
        return 0.0, 0.0, 0.0, abs(angular) <= 1e-6

    curvature = angular / velocity
    steering = math.atan(wheelbase * curvature)
    feasible = abs(steering) <= steering_limit
    return (velocity if feasible else 0.0), curvature, steering, feasible
