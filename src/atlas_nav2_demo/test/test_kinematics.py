import math
import pytest

from atlas_nav2_demo.kinematics import mix_twist, wheels_to_twist


RADIUS = 0.2
TRACK = 0.8


@pytest.mark.parametrize(('linear', 'angular', 'expected'), [
    (1.0, 0.0, (5.0, 5.0)),
    (-1.0, 0.0, (-5.0, -5.0)),
    (0.6, 0.5, (2.0, 4.0)),
    (0.6, -0.5, (4.0, 2.0)),
    (0.0, 1.0, (-2.0, 2.0)),
    (0.0, 0.0, (0.0, 0.0)),
])
def test_motion_cases(linear, angular, expected):
    speeds = mix_twist(linear, angular, TRACK, RADIUS)
    assert (speeds.front_left, speeds.rear_left) == pytest.approx((expected[0], expected[0]))
    assert (speeds.front_right, speeds.rear_right) == pytest.approx((expected[1], expected[1]))
    recovered = wheels_to_twist(expected[0], expected[1], TRACK, RADIUS)
    assert recovered == pytest.approx((linear, angular))


@pytest.mark.parametrize(('track', 'radius'), [(0.0, 0.2), (-1.0, 0.2), (0.8, 0.0)])
def test_invalid_geometry_rejected(track, radius):
    with pytest.raises(ValueError):
        mix_twist(0.0, 0.0, track, radius)


def test_nonfinite_command_rejected():
    with pytest.raises(ValueError):
        mix_twist(math.nan, 0.0, TRACK, RADIUS)
