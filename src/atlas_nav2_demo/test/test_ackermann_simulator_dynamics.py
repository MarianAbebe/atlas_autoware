"""Deterministic checks for the simulator's integration-time dynamics contract."""
import pytest

from atlas_nav2_demo.ackermann_kinematics import AckermannState, step


@pytest.mark.parametrize('dt', [0.001, 0.007, 0.02, 0.037, 0.05])
def test_irregular_simulation_steps_respect_acceleration_limits(dt):
    state = AckermannState()
    acceleration_limit = 0.8
    deceleration_limit = 1.2
    wheelbase = 1.2
    steering_limit = 0.5
    steering_rate_limit = 0.6
    speed_limit = 0.8

    for _ in range(400):
        target = 0.8
        before = state.speed
        state, _ = step(state, target, steering_limit, dt, wheelbase,
                        steering_limit, steering_rate_limit,
                        acceleration_limit, deceleration_limit, speed_limit)
        measured_acceleration = (state.speed - before) / dt
        assert measured_acceleration <= acceleration_limit + 1.0e-12
        assert measured_acceleration >= -deceleration_limit - 1.0e-12

    for _ in range(400):
        before = state.speed
        state, _ = step(state, 0.0, 0.0, dt, wheelbase, steering_limit,
                        steering_rate_limit, acceleration_limit,
                        deceleration_limit, speed_limit)
        measured_acceleration = (state.speed - before) / dt
        assert measured_acceleration <= acceleration_limit + 1.0e-12
        assert measured_acceleration >= -deceleration_limit - 1.0e-12
    assert state.speed == pytest.approx(0.0, abs=1.0e-12)
