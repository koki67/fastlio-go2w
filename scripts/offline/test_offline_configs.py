"""Verify that offline FAST-LIO configs preserve interactive tuning."""

from pathlib import Path

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


def test_offline_config_disables_only_unneeded_publishers():
    """Offline variants disable exactly three result-only publishers."""
    interactive = _parameters('mid360_go2w_accuracy_dense_false.yaml')
    offline = _parameters('mid360_go2w_accuracy_offline.yaml')

    assert _differences(interactive, offline) == PUBLISH_DIFFERENCES
