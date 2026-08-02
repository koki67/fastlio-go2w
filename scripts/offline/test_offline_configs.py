"""Verify that offline FAST-LIO configs and profiles preserve their contracts."""

import ast
from pathlib import Path

import pytest

import yaml


CONFIG_DIR = (
    Path(__file__).resolve().parents[2]
    / 'humble_ws'
    / 'src'
    / 'fastlio_go2w_bringup'
    / 'config'
)
MULTILIDAR_LAUNCH = (
    CONFIG_DIR.parent / 'launch' / 'offline_multilidar.launch.py'
)
MULTILIDAR_RUNNER = (
    Path(__file__).resolve().parent / 'run_multilidar_experiment.sh'
)
PUBLISH_DIFFERENCES = {
    ('publish', 'map_en'): (True, False),
    ('publish', 'path_en'): (True, False),
    ('publish', 'scan_bodyframe_pub_en'): (True, False),
}


def _parameters(filename):
    contents = (CONFIG_DIR / filename).read_text(encoding='utf-8')
    document = yaml.safe_load(contents)
    return document['/**']['ros__parameters']


def _flatten(value, prefix=()):
    if isinstance(value, dict):
        flattened = {}
        for key, child in value.items():
            flattened.update(_flatten(child, prefix + (key,)))
        return flattened
    return {prefix: value}


def _differences(left, right):
    left_flat = _flatten(left)
    right_flat = _flatten(right)
    all_paths = left_flat.keys() | right_flat.keys()
    return {
        path: (left_flat.get(path), right_flat.get(path))
        for path in all_paths
        if left_flat.get(path) != right_flat.get(path)
    }


@pytest.mark.parametrize(
    ('interactive_filename', 'offline_filename'),
    [
        (
            'mid360_go2w_accuracy_dense_false.yaml',
            'mid360_go2w_accuracy_offline.yaml',
        ),
        (
            'mid360_xt16_fused_accuracy_dense_false.yaml',
            'mid360_xt16_fused_accuracy_offline.yaml',
        ),
    ],
)
def test_offline_configs_disable_only_unneeded_publishers(
    interactive_filename, offline_filename
):
    """Offline variants disable exactly three result-only publishers."""
    interactive = _parameters(interactive_filename)
    offline = _parameters(offline_filename)

    assert _differences(interactive, offline) == PUBLISH_DIFFERENCES


def test_baseline_and_fused_offline_configs_differ_only_by_lidar_shape():
    """Baseline and fused variants share all tuning parameters."""
    baseline = _parameters('mid360_go2w_accuracy_offline.yaml')
    fused = _parameters('mid360_xt16_fused_accuracy_offline.yaml')

    assert _differences(baseline, fused) == {
        ('common', 'lid_topic'): ('/livox/lidar', '/livox/lidar_fused'),
        ('preprocess', 'scan_line'): (4, 20),
    }


def test_fused_full_profile_uses_every_eligible_point_and_firing_group():
    """The named full-density profile must keep both selection strides at one."""
    tree = ast.parse(MULTILIDAR_LAUNCH.read_text(encoding='utf-8'))
    assignments = {
        target.id: ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name)
    }

    assert assignments['_PROFILES']['fused-full'] == {
        'mid_point_stride': 1,
        'hesai_firing_stride': 1,
    }


def test_multilidar_default_run_name_starts_with_profile():
    """Default artifact names make the fusion method sortable at a glance."""
    runner = MULTILIDAR_RUNNER.read_text(encoding='utf-8')

    assert (
        '${FASTLIO_RESULTS_ROOT}/multilidar/$BAG_NAME/'
        '${PROFILE}-${RUN_STAMP}'
    ) in runner
    assert '${RUN_STAMP}-${PROFILE}' not in runner
