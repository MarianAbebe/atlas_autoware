#!/usr/bin/env python3
"""Guard against diagnostics accidentally changing the live ROS graph."""
import ast
from pathlib import Path


SOURCE = Path(__file__).with_name('run_lap_validation.py')


def method_calls(method, attribute):
    return [node for node in ast.walk(method) if isinstance(node, ast.Call) and
            isinstance(node.func, ast.Attribute) and node.func.attr == attribute]


def test_clock_sampling_does_not_create_ros_entities():
    tree = ast.parse(SOURCE.read_text())
    trial = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Trial')
    methods = {node.name: node for node in trial.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}
    assert 'sample_clocks' in methods and '__init__' in methods
    assert not method_calls(methods['sample_clocks'], 'create_subscription')
    assert not method_calls(methods['sample_clocks'], 'create_publisher')
    assert len(method_calls(methods['__init__'], 'create_subscription')) >= 10


def test_clock_sample_stores_distinct_time_domains():
    tree = ast.parse(SOURCE.read_text())
    trial = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Trial')
    method = next(node for node in trial.body if isinstance(node, ast.FunctionDef) and node.name == 'sample_clocks')
    assigned = {target.id for node in ast.walk(method) if isinstance(node, ast.Assign)
                for target in node.targets if isinstance(target, ast.Name)}
    assert {'mono_ns', 'ros_ns', 'wall_ns'} <= assigned


def test_trajectory_stage_capture_is_throttled_before_path_processing():
    tree = ast.parse(SOURCE.read_text())
    trial = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Trial')
    method = next(node for node in trial.body if isinstance(node, ast.FunctionDef)
                  and node.name == 'on_trajectory_stage')
    points_access = next(node for node in ast.walk(method)
                         if isinstance(node, ast.Attribute) and node.attr == 'points')
    throttle_return = next(node for node in ast.walk(method)
                           if isinstance(node, ast.Return) and node.lineno < points_access.lineno)
    assert throttle_return.lineno < points_access.lineno


def test_runtime_log_reader_advances_offset_instead_of_rereading():
    tree = ast.parse(SOURCE.read_text())
    trial = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Trial')
    spin = next(node for node in trial.body if isinstance(node, ast.FunctionDef) and node.name == 'spin')
    calls = [node for node in ast.walk(spin) if isinstance(node, ast.Call) and
             isinstance(node.func, ast.Attribute)]
    assert any(node.func.attr == 'seek' for node in calls)
    assert any(node.func.attr == 'read' for node in calls)


def test_perception_probe_covers_cloud_detection_prediction_and_cluster_timing():
    source = SOURCE.read_text()
    for topic in ('lidar/raw', 'lidar/extended', 'lidar/deskewed', 'lidar/compact',
                  'obstacles/pointcloud', 'clusters', 'obstacles/detected_objects',
                  'obstacles/predicted_objects'):
        assert topic in source
    for metric in ('cyclic_time_ms', 'processing_time_ms', 'pipeline_latency_ms'):
        assert metric in source
    assert '/proc/pressure/{resource}' in source
    assert "'memory_available_bytes'" in source
