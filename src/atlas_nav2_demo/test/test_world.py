import pytest

from atlas_nav2_demo.world import clearance, occupied, rectangles


def test_avoidable_geometry_blocks_nominal_path_and_has_clear_detour():
    assert occupied(2.0, 0.0, 'avoidable')
    assert clearance(2.0, 0.0, 0.32, 'avoidable') < 0.0
    assert clearance(2.0, 1.0, 0.32, 'avoidable') > 0.0


def test_blocked_wall_has_no_open_crossing():
    assert len(rectangles('blocked')) == 1
    assert occupied(2.0, 2.0, 'blocked')
    assert clearance(2.0, 0.0, 0.32, 'blocked') < 0.0


def test_invalid_scenario_rejected():
    with pytest.raises(ValueError):
        rectangles('not-a-scenario')
