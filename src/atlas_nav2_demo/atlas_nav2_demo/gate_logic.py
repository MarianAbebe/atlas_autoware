"""Pure safety-gate decision logic for deterministic unit testing."""
import math


def gated_velocity(enabled, emergency_stop, command_age, timeout,
                   linear, angular, max_linear, max_angular):
    values = (command_age, timeout, linear, angular, max_linear, max_angular)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('gate inputs must be finite')
    if timeout <= 0.0 or max_linear <= 0.0 or max_angular <= 0.0:
        raise ValueError('gate limits must be positive')
    if not enabled or emergency_stop or command_age > timeout:
        return 0.0, 0.0
    return (max(-max_linear, min(max_linear, linear)),
            max(-max_angular, min(max_angular, angular)))
