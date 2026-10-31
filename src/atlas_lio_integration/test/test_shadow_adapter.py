import copy
import math

import pytest
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

from atlas_lio_integration.shadow import (
    REJECTION_KEYS,
    ShadowOdometryFilter,
    make_diagnostic_array,
    validate_shadow_parameters,
)


def odometry(sec=1, nanosec=0):
    msg = Odometry()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.orientation.w = 1.0
    msg.pose.pose.position.x = 1.25
    msg.twist.twist.linear.x = 0.4
    msg.pose.covariance[0] = 0.01
    msg.twist.covariance[35] = 0.02
    return msg


def new_filter(**kwargs):
    return ShadowOdometryFilter(**kwargs)


def test_first_and_increasing_unique_samples_are_deep_copied_unchanged():
    filt = new_filter()
    first = odometry(10, 0)
    first_snapshot = copy.deepcopy(first)
    accepted, reason = filt.ingest(first, 5.0)
    assert reason is None
    assert accepted == first_snapshot
    assert accepted is not first
    accepted.pose.pose.position.x = 8.0
    assert first.pose.pose.position.x == first_snapshot.pose.pose.position.x

    second = odometry(10, 100_000_000)
    accepted, reason = filt.ingest(second, 5.1)
    assert reason is None
    assert accepted.header.stamp == second.header.stamp
    assert filt.accepted_unique_messages == 2


def test_duplicate_timestamp_rejected():
    filt = new_filter()
    assert filt.ingest(odometry(2, 0), 1.0)[1] is None
    assert filt.ingest(odometry(2, 0), 1.1)[1] == 'duplicate_timestamp_rejections'
    assert filt.rejections['duplicate_timestamp_rejections'] == 1


def test_regressing_timestamp_rejected():
    filt = new_filter()
    filt.ingest(odometry(3, 0), 1.0)
    assert filt.ingest(odometry(2, 999_999_999), 1.1)[1] == 'regressing_timestamp_rejections'


@pytest.mark.parametrize('sec,nanosec', [(0, 0), (-1, 0), (1, 1_000_000_000)])
def test_zero_or_invalid_timestamp_rejected(sec, nanosec):
    filt = new_filter()
    assert filt.ingest(odometry(sec, nanosec), 1.0)[1] == 'invalid_timestamp_rejections'


@pytest.mark.parametrize('field,value', [
    ('position', math.nan), ('position', math.inf),
    ('orientation', math.nan), ('orientation', 0.0), ('orientation', 2.0),
])
def test_invalid_pose_rejected(field, value):
    msg = odometry()
    if field == 'position':
        msg.pose.pose.position.y = value
    elif value == 0.0:
        msg.pose.pose.orientation.w = 0.0
    elif value == 2.0:
        msg.pose.pose.orientation.w = 2.0
    else:
        msg.pose.pose.orientation.z = value
    assert new_filter().ingest(msg, 1.0)[1] == 'invalid_pose_rejections'


def test_nonfinite_twist_rejected():
    msg = odometry()
    msg.twist.twist.angular.z = math.inf
    assert new_filter().ingest(msg, 1.0)[1] == 'invalid_twist_rejections'


@pytest.mark.parametrize('target', ['pose', 'twist'])
def test_nonfinite_covariance_rejected(target):
    msg = odometry()
    getattr(msg, target).covariance[7] = math.nan
    assert new_filter().ingest(msg, 1.0)[1] == 'invalid_covariance_rejections'


@pytest.mark.parametrize('field,value', [('header.frame_id', 'map'), ('child_frame_id', 'base_footprint')])
def test_frame_mismatch_rejected(field, value):
    msg = odometry()
    if field == 'header.frame_id':
        msg.header.frame_id = value
    else:
        msg.child_frame_id = value
    assert new_filter().ingest(msg, 1.0)[1] == 'frame_mismatch_rejections'


def test_duplicate_receipt_does_not_refresh_unique_state_freshness():
    filt = new_filter(stale_after_sec=0.15)
    filt.ingest(odometry(10), 4.0)
    filt.ingest(odometry(10), 4.1)
    level, message, values = filt.diagnostic_values(4.2, 10.2)
    assert values['message_receipt_age_sec'] == pytest.approx(0.1)
    assert values['unique_state_receipt_age_sec'] == pytest.approx(0.2)
    assert values['latest_estimator_header_age_sec'] == pytest.approx(0.2)
    assert level == 2
    assert message == 'unique estimator state is stale'


def test_new_unique_odometry_recovers_stale_state_diagnostic():
    filt = new_filter(stale_after_sec=0.15)
    assert filt.ingest(odometry(10), 4.0)[1] is None
    level, _, _ = filt.diagnostic_values(4.2, 10.2)
    assert level == 2

    accepted, reason = filt.ingest(odometry(10, 100_000_000), 4.25)
    level, message, values = filt.diagnostic_values(4.26, 10.2)
    assert reason is None and accepted is not None
    assert level == 0 and message == 'healthy'
    assert values['unique_state_receipt_age_sec'] == pytest.approx(0.01)


def test_unique_rate_and_maximum_timestamp_gap():
    filt = new_filter()
    for stamp, receipt in [(10.0, 2.0), (10.1, 2.1), (10.3, 2.3)]:
        sec = int(stamp)
        ns = round((stamp - sec) * 1e9)
        assert filt.ingest(odometry(sec, ns), receipt)[1] is None
    _, _, values = filt.diagnostic_values(2.4, 10.4)
    assert values['unique_output_rate_hz'] == pytest.approx(2 / 0.3)
    assert values['max_unique_timestamp_gap_sec'] == pytest.approx(0.2)
    assert values['max_unique_receipt_gap_sec'] == pytest.approx(0.2)


def test_rejected_messages_update_receipt_but_not_unique_receipt_interval():
    filt = new_filter()
    filt.ingest(odometry(10), 1.0)
    filt.ingest(odometry(10), 1.1)
    filt.ingest(odometry(10, 100_000_000), 1.2)
    _, _, values = filt.diagnostic_values(1.3, 10.3)
    assert values['max_message_receipt_gap_sec'] == pytest.approx(0.1)
    assert values['last_unique_receipt_gap_sec'] == pytest.approx(0.2)


def test_empty_diagnostic_clock_age_unavailable_and_state_is_bounded():
    filt = new_filter()
    level, _, values = filt.diagnostic_values(1.0, math.nan)
    assert level == 2
    assert values['message_receipt_age_sec'] is None
    assert values['latest_estimator_header_age_sec'] is None
    assert set(filt.rejections) == set(REJECTION_KEYS)
    assert not hasattr(filt, 'messages')
    assert not hasattr(filt, 'history')


def test_standard_diagnostic_message_contains_bounded_counters_and_clock_domains():
    filt = new_filter()
    stamp = Time(sec=20, nanosec=10)
    array = make_diagnostic_array(stamp, 1, 'filtered duplicate', {
        'total_received_messages': 2,
        'duplicate_timestamp_rejections': 1,
        'message_receipt_age_sec': 0.1,
        'latest_estimator_header_age_sec': None,
    })
    assert array.header.stamp == stamp
    assert len(array.status) == 1
    status = array.status[0]
    assert status.name == 'atlas_lio/shadow_odometry'
    assert status.level == b'\x01'
    assert {item.key: item.value for item in status.values} == {
        'total_received_messages': '2',
        'duplicate_timestamp_rejections': '1',
        'message_receipt_age_sec': '0.1',
        'latest_estimator_header_age_sec': 'unavailable',
    }


def test_shadow_launch_starts_only_the_adapter():
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / 'launch' / 'fastlio_shadow.launch.py'
    spec = importlib.util.spec_from_file_location('atlas_fastlio_shadow_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    assert len(entities) == 1
    node = entities[0]
    assert node.node_package == 'atlas_lio_integration'
    assert node.node_executable == 'lio_shadow_adapter'


@pytest.mark.parametrize('frame,child,rate,stale', [
    ('', 'base_link', 2.0, 0.3), ('odom', '', 2.0, 0.3),
    ('odom', 'base_link', math.inf, 0.3), ('odom', 'base_link', 21.0, 0.3),
    ('odom', 'base_link', 0.0, 0.3), ('odom', 'base_link', 2.0, math.nan),
])
def test_parameter_validation(frame, child, rate, stale):
    with pytest.raises(ValueError):
        validate_shadow_parameters(frame, child, rate, stale)
