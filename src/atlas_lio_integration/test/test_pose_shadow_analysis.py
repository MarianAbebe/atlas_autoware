import copy

import pytest

from atlas_lio_integration.pose_shadow_analysis import (
    analyze_streams,
    diagnostic_level_to_int,
)


POSE = {
    'position': {'x': 1.25, 'y': -2.0, 'z': 0.125},
    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.1, 'w': 0.99498743710662},
}


def raw(sec, receipt, pose=POSE, frame='odom', child='base_link'):
    return {
        'stamp': {'sec': sec, 'nanosec': 123},
        'receipt_monotonic_ns': receipt,
        'frame_id': frame,
        'child_frame_id': child,
        'pose': copy.deepcopy(pose),
    }


def shadow(sec, receipt, pose=POSE, frame='odom'):
    result = raw(sec, receipt, pose, frame)
    result.update({
        'message_type': 'geometry_msgs/msg/PoseStamped',
        'has_twist': False,
        'has_covariance': False,
    })
    return result


def streams(raw_rows, shadow_rows):
    return analyze_streams(raw_rows, shadow_rows, [], [raw(1, 1)])


def test_receipt_window_boundary_does_not_reject_exact_forwarding():
    source = raw(10, 130_236_886_660_450)
    forwarded = shadow(10, 130_236_887_354_319)
    report = streams([source], [forwarded])
    assert report['checks']['recorded_forwarding_exact']
    assert report['metrics']['raw_samples_outside_receipt_overlap'] == 1


def test_exact_forwarding_matches_full_unique_timestamp_sets_and_pose():
    source = [raw(10, 100), raw(10, 101), raw(11, 200)]
    forwarded = [shadow(10, 110), shadow(11, 210)]
    report = streams(source, forwarded)
    assert report['checks']['recorded_unique_timestamp_sets_match']
    assert report['checks']['recorded_forwarding_exact']
    assert report['metrics']['raw_duplicate_count'] == 1
    assert report['metrics']['pose_mismatch_count'] == 0


def test_missing_source_timestamp_fails():
    report = streams([raw(10, 100)], [shadow(10, 110), shadow(11, 120)])
    assert not report['checks']['recorded_unique_timestamp_sets_match']
    assert not report['checks']['recorded_forwarding_exact']
    assert report['metrics']['missing_raw_timestamps_for_shadow'] == 1


def test_missing_shadow_timestamp_fails():
    report = streams([raw(10, 100), raw(11, 120)], [shadow(10, 110)])
    assert not report['checks']['recorded_unique_timestamp_sets_match']
    assert not report['checks']['recorded_forwarding_exact']
    assert report['metrics']['missing_shadow_timestamps_for_raw'] == 1


def test_pose_mismatch_fails_even_when_stamp_sets_match():
    altered = copy.deepcopy(POSE)
    altered['position']['x'] += 1e-12
    report = streams([raw(10, 100)], [shadow(10, 110, altered)])
    assert report['checks']['recorded_unique_timestamp_sets_match']
    assert not report['checks']['recorded_forwarding_exact']
    assert report['metrics']['pose_mismatch_count'] == 1


@pytest.mark.parametrize('rows', [
    [shadow(10, 100), shadow(10, 110)],
    [shadow(11, 100), shadow(10, 110)],
])
def test_duplicate_or_regressing_shadow_timestamps_fail(rows):
    source = [raw(10, 90), raw(11, 95)]
    assert not streams(source, rows)['checks']['recorded_forwarding_exact']


def test_empty_raw_or_shadow_stream_fails_without_vacuous_pass():
    assert not streams([], [shadow(10, 100)])['checks']['recorded_forwarding_exact']
    assert not streams([raw(10, 100)], [])['checks']['recorded_forwarding_exact']


def test_insufficient_samples_report_missing_gap_and_rate_as_none():
    report = streams([raw(10, 100)], [shadow(10, 110)])
    metrics = report['metrics']
    assert metrics['max_raw_unique_header_gap_sec'] is None
    assert metrics['max_shadow_unique_header_gap_sec'] is None
    assert metrics['max_raw_monotonic_receipt_gap_sec'] is None
    assert metrics['max_shadow_monotonic_receipt_gap_sec'] is None
    assert metrics['raw_unique_rate_hz_in_receipt_overlap'] is None
    assert metrics['shadow_rate_hz_in_receipt_overlap'] is None


@pytest.mark.parametrize('value,expected', [
    (b'\x00', 0), (b'\x01', 1), (b'\x02', 2), (b'\x03', 3),
    (bytearray(b'\x00'), 0), (bytearray(b'\x01'), 1),
    (bytearray(b'\x02'), 2), (bytearray(b'\x03'), 3),
    (0, 0), (3, 3),
])
def test_diagnostic_uint8_levels_serialize_as_integers(value, expected):
    assert diagnostic_level_to_int(value) == expected


def test_invalid_diagnostic_level_buffer_rejected():
    with pytest.raises(ValueError):
        diagnostic_level_to_int(b'')
