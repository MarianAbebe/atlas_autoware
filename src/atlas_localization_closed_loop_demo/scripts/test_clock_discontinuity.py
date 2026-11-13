#!/usr/bin/env python3
"""Deterministic clock-step tests; no system clock is changed."""
import ast
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import localization_watchdog as localization


def test_safety_tick_defers_fault_logging_to_diagnostic_callback():
    source = Path(localization.existing.__file__).read_text()
    tree = ast.parse(source)
    watchdog = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                    and node.name == 'Watchdog')
    methods = {node.name: node for node in watchdog.body
               if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}
    tick_calls = [node for node in ast.walk(methods['tick']) if isinstance(node, ast.Call)]
    assert not any(isinstance(node.func, ast.Attribute) and node.func.attr == 'warn'
                   for node in tick_calls)
    assert any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
               and node.func.id == 'format_fault_detail'
               for node in ast.walk(methods['publish_status_tick']))


def detector(tolerance=0.1):
    return localization.existing.ClockDiscontinuityMonitor(tolerance)


def test_forward_and_backward_ros_clock_steps_are_detected():
    for jumped_ros_ns in (1_172_000_000_000, -1_000_000):
        monitor = detector()
        assert monitor.observe(10_000_000_000, 20_000_000_000) is None
        sample = monitor.observe(jumped_ros_ns, 20_020_000_000)
        assert sample is not None
        assert abs(sample['delta_discrepancy_sec']) > sample['tolerance_sec']


def test_scheduler_delay_with_matching_ros_and_monotonic_deltas_is_not_a_clock_step():
    monitor = detector()
    assert monitor.observe(10_000_000_000, 20_000_000_000) is None
    assert monitor.observe(10_800_000_000, 20_800_000_000) is None


def test_clock_step_latches_watchdog_and_does_not_auto_rearm():
    cls = localization.existing.Watchdog
    node = object.__new__(cls)
    node.state_lock = threading.RLock()
    node.clock_monitor = detector()
    node.clock_monitor.observe(10_000_000_000, 20_000_000_000)
    node.clock_discontinuity = None
    node.last_tick = None
    node.max_tick_gap_sec = 0.0
    node.tick_count = 0
    node.last_tick_start_monotonic = None
    node.last_tick_gap_sec = None
    node.last_tick_duration_sec = None
    node.last_stale = []
    node.ready = True
    node.bad = False
    node.input_health = {}
    node.fault_at = None
    node.fault_inputs = []
    node.fault_health = None
    node.fault_tick_gap = None
    node.pub = SimpleNamespace(publish=lambda _msg: (_ for _ in ()).throw(
        AssertionError('heartbeat must not publish after clock step')))
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_182_000_000_000))
    node.stale = lambda: []
    node.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)

    with patch.object(localization.existing.time, 'monotonic', return_value=20.02), \
         patch.object(localization.existing.time, 'monotonic_ns', return_value=20_020_000_000):
        cls.tick(node)
    assert node.bad is True
    assert node.fault_inputs == ['clock_discontinuity']
    assert node.clock_discontinuity is not None

    # Clock stabilizes and the test input is fresh, but the latch stays set.
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_183_000_000_000))
    with patch.object(localization.existing.time, 'monotonic', return_value=21.02), \
         patch.object(localization.existing.time, 'monotonic_ns', return_value=21_020_000_000):
        cls.tick(node)
    assert node.bad is True
    assert node.ready is True


def test_clock_latch_can_only_be_cleared_by_explicit_stopped_rearm():
    node = object.__new__(localization.existing.Watchdog)
    node.state_lock = threading.RLock()
    node.bad = True
    node.stationary_since = 1.0
    node.stale = lambda: []
    response = SimpleNamespace(success=False, message='')
    with patch.object(localization.existing.time, 'monotonic', return_value=5.0):
        localization.existing.Watchdog.rearm(node, None, response)
    assert response.success is True
    assert node.bad is False
    assert node.ready is False
    assert node.good_since is None
