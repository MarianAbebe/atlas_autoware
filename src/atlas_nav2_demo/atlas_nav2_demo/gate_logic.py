"""Pure safety-gate decision logic for deterministic unit testing."""
import math


def gated_velocity(enabled, emergency_stop, command_age, localization_age, timeout,
                   linear, angular, max_linear, max_angular):
    """Return a bounded command only while command and localization are live.

    ``localization_age`` is receipt age in the gate's monotonic-clock domain.
    It deliberately does not reinterpret or replace an odometry source stamp:
    a stale or unavailable localization stream is a motion-inhibiting fault.
    """
    values = (command_age, localization_age, timeout, linear, angular,
              max_linear, max_angular)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('gate inputs must be finite')
    if timeout <= 0.0 or max_linear <= 0.0 or max_angular <= 0.0:
        raise ValueError('gate limits must be positive')
    if (not enabled or emergency_stop or command_age > timeout
            or localization_age > timeout):
        return 0.0, 0.0
    return (max(-max_linear, min(max_linear, linear)),
            max(-max_angular, min(max_angular, angular)))
