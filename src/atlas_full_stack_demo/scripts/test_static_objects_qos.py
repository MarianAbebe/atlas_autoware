#!/usr/bin/env python3
"""Protect the exact-stamp detection/membership pairing QoS contract."""
import ast
from pathlib import Path


SOURCE = Path(__file__).with_name('static_objects.py')


def test_membership_cloud_subscription_is_reliable_and_latest_only():
    tree = ast.parse(SOURCE.read_text())
    adapter = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                   and node.name == 'Adapter')
    init = next(node for node in adapter.body if isinstance(node, ast.FunctionDef)
                and node.name == '__init__')
    call = next(node for node in ast.walk(init) if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'create_subscription'
                and len(node.args) > 1 and isinstance(node.args[1], ast.Constant)
                and node.args[1].value == 'clusters')
    qos = call.args[3]
    assert isinstance(qos, ast.Call) and isinstance(qos.func, ast.Name)
    assert qos.func.id == 'QoSProfile'
    keywords = {kw.arg: kw.value for kw in qos.keywords}
    assert isinstance(keywords['depth'], ast.Constant) and keywords['depth'].value == 1
    assert isinstance(keywords['reliability'], ast.Attribute)
    assert keywords['reliability'].attr == 'RELIABLE'
