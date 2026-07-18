"""Verify that offline FAST-LIO configs preserve interactive tuning."""

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
