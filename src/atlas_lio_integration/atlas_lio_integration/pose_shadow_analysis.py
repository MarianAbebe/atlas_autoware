"""Offline validation for FAST-LIO pose-only shadow evidence JSONL streams.

Forwarding is keyed by ROS header timestamps and exact pose values. Monotonic
receipt time is reserved for capture overlap, rates, and receipt-gap metrics.
"""
import argparse
import json
import math
import os
from pathlib import Path


STREAM_FILES = (
    'raw_estimator_odometry.jsonl',
    'shadow_pose.jsonl',
    'shadow_pose_diagnostics.jsonl',
    'ground_truth.jsonl',
)
DIAGNOSTIC_COUNTERS = (
    'accepted_unique_pose_samples',
    'duplicate_timestamp_rejections',
    'regressing_timestamp_rejections',
    'invalid_timestamp_rejections',
    'invalid_position_rejections',
    'invalid_quaternion_rejections',
    'frame_mismatch_rejections',
)


def diagnostic_level_to_int(value):
    """Convert ROS diagnostic uint8 values represented as ints or one-byte buffers."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError('diagnostic status level buffer must contain exactly one byte')
        return value[0]
    result = int(value)
    if not 0 <= result <= 3:
        raise ValueError('diagnostic status level must be in [0, 3]')
    return result


def _stamp_ns(row):
    stamp = row['stamp']
    sec, nanosec = stamp['sec'], stamp['nanosec']
    if isinstance(sec, bool) or isinstance(nanosec, bool):
        raise ValueError('boolean timestamp component')
    if not isinstance(sec, int) or not isinstance(nanosec, int):
        raise ValueError('timestamp components must be integers')
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError('timestamp is outside ROS builtin_interfaces/Time range')
    return sec * 1_000_000_000 + nanosec


def _pose_equal(left, right):
    for group, axes in (('position', ('x', 'y', 'z')),
                        ('orientation', ('x', 'y', 'z', 'w'))):
        for axis in axes:
            if left[group][axis] != right[group][axis]:
                return False
    return True


def _finite_pose(pose):
    try:
        values = [float(pose['position'][axis]) for axis in ('x', 'y', 'z')]
        values.extend(float(pose['orientation'][axis]) for axis in ('x', 'y', 'z', 'w'))
        return all(math.isfinite(value) for value in values)
    except (KeyError, TypeError, ValueError):
        return False


def _gap(values):
    return max((later - earlier for earlier, later in zip(values, values[1:])), default=None)


def _unique_rate(rows):
    by_stamp = {}
    for row in rows:
        by_stamp[_stamp_ns(row)] = row
    ordered = sorted(by_stamp.values(), key=_stamp_ns)
    if len(ordered) < 2:
        return None
    duration = (_stamp_ns(ordered[-1]) - _stamp_ns(ordered[0])) * 1e-9
    return (len(ordered) - 1) / duration if duration > 0 else None


def _diagnostic_statuses(records):
    for record in records:
        for status in record.get('status', []):
            if status.get('name') == 'atlas_lio/shadow_pose':
                yield status.get('values', {})


def analyze_streams(raw, shadow, diagnostics, ground_truth):
    """Return machine-readable checks and metrics without mutating input streams."""
    raw_stamp_errors = []
    shadow_stamp_errors = []
    raw_stamps = []
    shadow_stamps = []
    for index, row in enumerate(raw):
        try:
            raw_stamps.append(_stamp_ns(row))
        except (KeyError, TypeError, ValueError) as error:
            raw_stamp_errors.append({'index': index, 'error': str(error)})
    for index, row in enumerate(shadow):
        try:
            shadow_stamps.append(_stamp_ns(row))
        except (KeyError, TypeError, ValueError) as error:
            shadow_stamp_errors.append({'index': index, 'error': str(error)})

    raw_set = set(raw_stamps)
    shadow_set = set(shadow_stamps)
    checks = {
        'raw_nonempty': bool(raw),
        'shadow_nonempty': bool(shadow),
        'ground_truth_nonempty': bool(ground_truth),
        'raw_timestamps_valid': len(raw_stamps) == len(raw) and not raw_stamp_errors,
        'shadow_timestamps_valid': len(shadow_stamps) == len(shadow) and not shadow_stamp_errors,
        'raw_timestamps_positive': bool(raw_stamps) and all(value > 0 for value in raw_stamps),
        'shadow_timestamps_positive': bool(shadow_stamps) and all(value > 0 for value in shadow_stamps),
        'shadow_timestamps_strict': bool(shadow_stamps) and all(
            later > earlier for earlier, later in zip(shadow_stamps, shadow_stamps[1:])
        ),
        'pose_output_schema_only': bool(shadow) and all(
            row.get('message_type') == 'geometry_msgs/msg/PoseStamped'
            and row.get('has_twist') is False and row.get('has_covariance') is False
            for row in shadow
        ),
        'raw_frames_match_contract': bool(raw) and all(
            row.get('frame_id') == 'odom' and row.get('child_frame_id') == 'base_link'
            for row in raw
        ),
        'shadow_frames_match_contract': bool(shadow) and all(
            row.get('frame_id') == 'odom' for row in shadow
        ),
        'recorded_unique_timestamp_sets_match': bool(raw and shadow) and raw_set == shadow_set,
    }

    missing_raw = shadow_set - raw_set
    missing_shadow = raw_set - shadow_set
    source_by_stamp = {}
    for row in raw:
        try:
            source_by_stamp.setdefault(_stamp_ns(row), []).append(row)
        except (KeyError, TypeError, ValueError):
            continue
    pose_mismatches = []
    for row in shadow:
        try:
            timestamp = _stamp_ns(row)
            pose = row['pose']
            sources = source_by_stamp.get(timestamp, [])
        except (KeyError, TypeError, ValueError):
            pose_mismatches.append(None)
            continue
        if not any(_pose_equal(pose, source.get('pose', {})) for source in sources):
            pose_mismatches.append(timestamp)

    checks['recorded_forwarding_exact'] = (
        checks['raw_nonempty'] and checks['shadow_nonempty']
        and checks['raw_timestamps_valid'] and checks['shadow_timestamps_valid']
        and checks['raw_timestamps_positive'] and checks['shadow_timestamps_positive']
        and checks['shadow_timestamps_strict']
        and checks['raw_frames_match_contract'] and checks['shadow_frames_match_contract']
        and checks['recorded_unique_timestamp_sets_match']
        and not pose_mismatches
    )

    # Receipt monotonic windows are used only for time/rate analysis. They do
    # not decide whether a pose was forwarded correctly.
    try:
        if raw and shadow:
            overlap_start = max(min(row['receipt_monotonic_ns'] for row in raw),
                                min(row['receipt_monotonic_ns'] for row in shadow))
            overlap_end = min(max(row['receipt_monotonic_ns'] for row in raw),
                              max(row['receipt_monotonic_ns'] for row in shadow))
        else:
            overlap_start, overlap_end = 0, -1
        raw_overlap = [row for row in raw if overlap_start <= row['receipt_monotonic_ns'] <= overlap_end]
        shadow_overlap = [row for row in shadow if overlap_start <= row['receipt_monotonic_ns'] <= overlap_end]
    except (KeyError, TypeError):
        overlap_start, overlap_end = 0, -1
        raw_overlap, shadow_overlap = [], []

    status_values = list(_diagnostic_statuses(diagnostics))
    latest = status_values[-1] if status_values else {}
    counters = {key: int(latest.get(key, '0')) for key in DIAGNOSTIC_COUNTERS}
    total_received = int(latest.get('total_received_messages', '0'))
    checks['diagnostics_show_acceptance'] = bool(status_values) and counters['accepted_unique_pose_samples'] > 0
    checks['diagnostic_counters_self_consistent'] = (
        bool(status_values)
        and total_received == counters['accepted_unique_pose_samples']
        + sum(counters[key] for key in DIAGNOSTIC_COUNTERS[1:])
    )
    checks['adapter_duplicate_rejections_observed'] = counters['duplicate_timestamp_rejections'] > 0

    invalid_positions = False
    invalid_quaternions = False
    for row in raw:
        pose = row.get('pose', {})
        try:
            invalid_positions |= not all(math.isfinite(float(pose['position'][axis])) for axis in ('x', 'y', 'z'))
            q = [float(pose['orientation'][axis]) for axis in ('x', 'y', 'z', 'w')]
            norm = math.sqrt(sum(value * value for value in q))
            invalid_quaternions |= (not all(math.isfinite(value) for value in q)
                                    or not math.isfinite(norm) or norm == 0
                                    or abs(norm - 1.0) > 0.01)
        except (KeyError, TypeError, ValueError):
            invalid_positions = invalid_quaternions = True
    checks['no_unexpected_invalid_position_rejections'] = invalid_positions or counters['invalid_position_rejections'] == 0
    checks['no_unexpected_invalid_quaternion_rejections'] = invalid_quaternions or counters['invalid_quaternion_rejections'] == 0

    freshness_pairs = []
    for values in status_values:
        try:
            raw_age = float(values['latest_raw_message_receipt_age_sec'])
            unique_age = float(values['latest_unique_state_receipt_age_sec'])
            if math.isfinite(raw_age) and math.isfinite(unique_age):
                freshness_pairs.append((raw_age, unique_age))
        except (KeyError, TypeError, ValueError):
            continue
    checks['duplicate_receipt_does_not_refresh_unique_age'] = any(
        raw_age + 0.005 < unique_age for raw_age, unique_age in freshness_pairs
    )

    raw_rate = _unique_rate(raw_overlap)
    shadow_rate = _unique_rate(shadow_overlap)
    checks['shadow_rate_tracks_raw_unique_rate'] = (
        raw_rate is not None and shadow_rate is not None
        and abs(shadow_rate - raw_rate) <= max(0.5, 0.1 * raw_rate)
    )
    diagnostic_rate = None
    if len(diagnostics) > 1:
        receipt_span = (diagnostics[-1].get('receipt_monotonic_ns', 0)
                        - diagnostics[0].get('receipt_monotonic_ns', 0)) * 1e-9
        if receipt_span > 0:
            diagnostic_rate = (len(diagnostics) - 1) / receipt_span

    metrics = {
        'raw_callback_count': len(raw),
        'raw_unique_timestamp_count': len(raw_set),
        'raw_duplicate_count': len(raw) - len(raw_set),
        'shadow_output_count': len(shadow),
        'shadow_unique_timestamp_count': len(shadow_set),
        'adapter_duplicate_rejection_count': counters['duplicate_timestamp_rejections'],
        'missing_raw_timestamps_for_shadow': len(missing_raw),
        'missing_shadow_timestamps_for_raw': len(missing_shadow),
        'pose_mismatch_count': len(pose_mismatches),
        'raw_unique_rate_hz_in_receipt_overlap': raw_rate,
        'shadow_rate_hz_in_receipt_overlap': shadow_rate,
        'max_raw_unique_header_gap_sec': _gap(sorted(raw_set)) * 1e-9 if len(raw_set) > 1 else None,
        'max_shadow_unique_header_gap_sec': _gap(sorted(shadow_set)) * 1e-9 if len(shadow_set) > 1 else None,
        'max_raw_monotonic_receipt_gap_sec': _gap([x['receipt_monotonic_ns'] for x in raw]) * 1e-9 if len(raw) > 1 else None,
        'max_shadow_monotonic_receipt_gap_sec': _gap([x['receipt_monotonic_ns'] for x in shadow]) * 1e-9 if len(shadow) > 1 else None,
        'diagnostic_publication_rate_hz': diagnostic_rate,
        'latest_raw_receipt_age_sec': latest.get('latest_raw_message_receipt_age_sec'),
        'latest_unique_pose_receipt_age_sec': latest.get('latest_unique_state_receipt_age_sec'),
        'receipt_overlap_monotonic_ns': [overlap_start, overlap_end] if overlap_end >= overlap_start else None,
        'raw_samples_outside_receipt_overlap': len(raw) - len(raw_overlap),
        'shadow_samples_outside_receipt_overlap': len(shadow) - len(shadow_overlap),
        'raw_invalid_timestamp_samples': raw_stamp_errors,
        'shadow_invalid_timestamp_samples': shadow_stamp_errors,
        'missing_raw_stamps': sorted(missing_raw),
        'missing_shadow_stamps': sorted(missing_shadow),
        'pose_mismatch_stamps': pose_mismatches,
        'ground_truth_count': len(ground_truth),
        'diagnostic_count': len(diagnostics),
        'adapter_counters': counters,
    }
    return {'result': 'PASS' if all(checks.values()) else 'FAIL',
            'checks': checks, 'failed_checks': [key for key, passed in checks.items() if not passed],
            'metrics': metrics}


def _read_jsonl(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def analyze_directory(input_dir):
    root = Path(input_dir).resolve()
    streams = [_read_jsonl(root / name) for name in STREAM_FILES]
    return analyze_streams(*streams)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evidence_dir', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True,
                        help='new directory for analysis output; existing paths are refused')
    args = parser.parse_args(argv)
    source = args.evidence_dir.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    if not source.is_dir():
        parser.error('evidence_dir must already exist')
    if os.path.commonpath((str(source), str(output))) == str(source):
        parser.error('output directory must be outside the preserved evidence directory')
    try:
        output.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        parser.error('output directory already exists; choose a unique path')
    result = analyze_directory(source)
    (output / 'validation.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    context = {'input_evidence_dir': str(source), 'output_dir': str(output),
               'analysis': 'offline exact timestamp/pose comparison; receipt time used only for timing'}
    (output / 'analysis_context.json').write_text(json.dumps(context, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['result'] == 'PASS' else 1
