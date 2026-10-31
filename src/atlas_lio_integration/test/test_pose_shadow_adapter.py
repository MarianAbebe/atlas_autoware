import copy
import math
from pathlib import Path
import importlib.util

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from atlas_lio_integration.pose_shadow import (
    REJECTION_KEYS,
    ShadowPoseFilter,
    make_pose_diagnostic_array,
    validate_pose_shadow_parameters,
)


def odometry(sec=1, nanosec=0):
    msg = Odometry()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = 1.25
    msg.pose.pose.position.y = -2.5
    msg.pose.pose.position.z = 0.125
    msg.pose.pose.orientation.x = 0.1
    msg.pose.pose.orientation.y = 0.2
    msg.pose.pose.orientation.z = 0.3
    msg.pose.pose.orientation.w = math.sqrt(0.86)
    return msg


def test_first_and_increasing_pose_samples_are_exact_deep_copies():
    filt = ShadowPoseFilter()
    source = odometry(10, 123_456_789)
    source_snapshot = copy.deepcopy(source.pose.pose)
    accepted, reason = filt.ingest(source, 5.0)

    assert reason is None
    assert isinstance(accepted, PoseStamped)
    assert accepted.header.stamp == source.header.stamp
    assert accepted.header.frame_id == 'odom'
    assert accepted.pose == source_snapshot
    assert accepted.pose is not source.pose.pose

    accepted.pose.position.x = 99.0
    assert source.pose.pose == source_snapshot
    source.pose.pose.position.y = 88.0
    assert accepted.pose.position.y == source_snapshot.position.y

    second, reason = filt.ingest(odometry(10, 223_456_789), 5.1)
    assert reason is None
    assert second.header.stamp.nanosec == 223_456_789
    assert filt.accepted_unique_pose_samples == 2


def test_duplicate_and_regressing_timestamps_are_rejected():
    filt = ShadowPoseFilter()
    assert filt.ingest(odometry(5, 20), 1.0)[1] is None
    assert filt.ingest(odometry(5, 20), 1.1)[1] == 'duplicate_timestamp_rejections'
    assert filt.ingest(odometry(5, 19), 1.2)[1] == 'regressing_timestamp_rejections'
    assert filt.rejections['duplicate_timestamp_rejections'] == 1
    assert filt.rejections['regressing_timestamp_rejections'] == 1


@pytest.mark.parametrize('sec,nanosec', [(0, 0), (-1, 0), (1, 1_000_000_000)])
def test_zero_or_invalid_timestamp_is_rejected(sec, nanosec):
    assert ShadowPoseFilter().ingest(odometry(sec, nanosec), 1.0)[1] == (
        'invalid_timestamp_rejections'
    )


@pytest.mark.parametrize('coordinate', ['x', 'y', 'z'])
def test_nonfinite_position_is_rejected(coordinate):
    msg = odometry()
    setattr(msg.pose.pose.position, coordinate, math.nan)
    assert ShadowPoseFilter().ingest(msg, 1.0)[1] == 'invalid_position_rejections'


@pytest.mark.parametrize('component,value', [
    ('x', math.inf), ('y', math.nan), ('z', 0.0), ('w', 0.0), ('w', 2.0),
])
def test_nonfinite_zero_norm_or_out_of_tolerance_quaternion_is_rejected(component, value):
    msg = odometry()
    for field in ('x', 'y', 'z', 'w'):
        setattr(msg.pose.pose.orientation, field, 0.0)
    setattr(msg.pose.pose.orientation, component, value)
    assert ShadowPoseFilter().ingest(msg, 1.0)[1] == 'invalid_quaternion_rejections'


def test_quaternion_tolerance_is_configurable_and_values_are_not_normalized():
    msg = odometry()
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.04
    accepted, reason = ShadowPoseFilter(quaternion_norm_tolerance=0.05).ingest(msg, 1.0)
    assert reason is None
    assert accepted.pose.orientation.w == 1.04

    assert ShadowPoseFilter(quaternion_norm_tolerance=0.02).ingest(msg, 1.0)[1] == (
        'invalid_quaternion_rejections'
    )


@pytest.mark.parametrize('field,value', [
    ('header.frame_id', 'map'), ('child_frame_id', 'base_footprint'),
])
def test_frame_mismatches_are_rejected(field, value):
    msg = odometry()
    if field == 'header.frame_id':
        msg.header.frame_id = value
    else:
        msg.child_frame_id = value
    assert ShadowPoseFilter().ingest(msg, 1.0)[1] == 'frame_mismatch_rejections'


@pytest.mark.parametrize('bad_field', [
    'twist_value', 'twist_covariance', 'pose_covariance',
])
def test_pose_only_filter_ignores_twist_and_both_covariance_arrays(bad_field):
    msg = odometry()
    if bad_field == 'twist_value':
        msg.twist.twist.angular.z = math.inf
    elif bad_field == 'twist_covariance':
        msg.twist.covariance[1] = math.nan
    else:
        msg.pose.covariance[0] = -math.inf

    accepted, reason = ShadowPoseFilter().ingest(msg, 1.0)
    assert reason is None
    assert accepted.pose == msg.pose.pose


def test_duplicate_receipts_refresh_raw_age_but_not_unique_state_age():
    filt = ShadowPoseFilter(stale_after_sec=0.15)
    filt.ingest(odometry(10), 4.0)
    filt.ingest(odometry(10), 4.1)
    level, message, values = filt.diagnostic_values(4.2)
    assert values['latest_raw_message_receipt_age_sec'] == pytest.approx(0.1)
    assert values['latest_unique_state_receipt_age_sec'] == pytest.approx(0.2)
    assert level == 2
    assert message == 'unique estimator pose is stale'


def test_unique_pose_after_dropout_restores_fresh_state():
    filt = ShadowPoseFilter(stale_after_sec=0.15)
    assert filt.ingest(odometry(10), 4.0)[1] is None
    level, _, values = filt.diagnostic_values(4.2)
    assert level == 2
    assert values['latest_unique_state_receipt_age_sec'] == pytest.approx(0.2)

    accepted, reason = filt.ingest(odometry(10, 100_000_000), 4.25)
    level, message, values = filt.diagnostic_values(4.26)
    assert reason is None and accepted is not None
    assert level == 0 and message == 'healthy'
    assert values['latest_unique_state_receipt_age_sec'] == pytest.approx(0.01)


def test_unique_rate_and_maximum_header_gap_use_only_accepted_stamps():
    filt = ShadowPoseFilter()
    for (sec, nanosec), receipt in [
        ((10, 0), 2.0), ((10, 100_000_000), 2.1), ((10, 300_000_000), 2.3),
    ]:
        assert filt.ingest(odometry(sec, nanosec), receipt)[1] is None
    filt.ingest(odometry(10, 300_000_000), 2.4)  # duplicate does not alter span/gap
    _, _, values = filt.diagnostic_values(2.5)
    assert values['unique_pose_publication_rate_hz'] == pytest.approx(2 / 0.3)
    assert values['max_unique_header_gap_sec'] == pytest.approx(0.2)
    assert filt.accepted_unique_pose_samples == 3


def test_diagnostics_are_bounded_and_standard_message_has_pose_fields():
    filt = ShadowPoseFilter()
    for i in range(1000):
        filt.ingest(odometry(i + 1), float(i))
    filt.ingest(odometry(1000), 1000.0)
    _, _, values = filt.diagnostic_values(1000.5)
    assert values['total_received_messages'] == 1001
    assert values['accepted_unique_pose_samples'] == 1000
    assert set(filt.rejections) == set(REJECTION_KEYS)
    assert not hasattr(filt, 'messages')
    assert not hasattr(filt, 'history')

    array = make_pose_diagnostic_array(
        odometry(20).header.stamp, 1, 'filtered inputs', values
    )
    assert array.header.stamp == odometry(20).header.stamp
    assert array.status[0].name == 'atlas_lio/shadow_pose'
    fields = {item.key: item.value for item in array.status[0].values}
    assert fields['accepted_unique_pose_samples'] == '1000'
    assert fields['latest_raw_message_receipt_age_sec'] == '0.5'
    assert fields['latest_unique_state_receipt_age_sec'] == '1.5'


@pytest.mark.parametrize('rate,stale,tolerance', [
    (math.inf, 0.3, 0.01), (2.0, 0.0, 0.01), (2.0, 0.3, math.nan),
    (2.0, 0.3, 1.0),
])
def test_pose_shadow_parameter_validation(rate, stale, tolerance):
    with pytest.raises(ValueError):
        validate_pose_shadow_parameters('odom', 'base_link', rate, stale, tolerance)


def test_standalone_launch_contains_only_pose_shadow_adapter():
    path = Path(__file__).resolve().parents[1] / 'launch' / 'fastlio_pose_shadow.launch.py'
    spec = importlib.util.spec_from_file_location('atlas_fastlio_pose_shadow_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes = module.generate_launch_description().entities
    assert len(nodes) == 1
    assert nodes[0].node_package == 'atlas_lio_integration'
    assert nodes[0].node_executable == 'lio_pose_shadow_adapter'
