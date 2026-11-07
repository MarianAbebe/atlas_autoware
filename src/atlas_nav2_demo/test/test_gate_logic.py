import pytest

from atlas_nav2_demo.gate_logic import gated_velocity


def test_default_disabled_is_zero():
    assert gated_velocity(False, False, 0.0, 0.0, 0.25, 0.4, 0.5, 0.5, 1.0) == (0.0, 0.0)


def test_stale_command_is_zero():
    assert gated_velocity(True, False, 0.251, 0.0, 0.25, 0.4, 0.5, 0.5, 1.0) == (0.0, 0.0)


def test_stale_localization_is_zero_even_with_a_fresh_command():
    assert gated_velocity(True, False, 0.0, 0.251, 0.25, 0.4, 0.5, 0.5, 1.0) == (0.0, 0.0)


def test_emergency_stop_is_zero():
    assert gated_velocity(True, True, 0.0, 0.0, 0.25, 0.4, 0.5, 0.5, 1.0) == (0.0, 0.0)


def test_velocity_limits_are_enforced():
    assert gated_velocity(True, False, 0.0, 0.0, 0.25, 2.0, -3.0, 0.5, 1.0) == (0.5, -1.0)


def test_invalid_limit_rejected():
    with pytest.raises(ValueError):
        gated_velocity(True, False, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0)
