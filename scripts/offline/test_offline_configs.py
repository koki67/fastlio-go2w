"""Verify that offline FAST-LIO configs preserve interactive tuning."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import yaml


CONFIG_DIR = (
    Path(__file__).resolve().parents[2]
    / 'humble_ws'
    / 'src'
    / 'fastlio_go2w_bringup'
    / 'config'
)
REPO_ROOT = Path(__file__).resolve().parents[2]
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


def test_xt16_fastlio_contract_and_extrinsics_match_calibration():
    calibration = yaml.safe_load(
        (REPO_ROOT / 'config/sensor/go2w_xt16_calibration.yaml').read_text(
            encoding='utf-8'
        )
    )
    parameters = _parameters('xt16_go2w_accuracy_offline.yaml')
    lidar_to_imu = calibration['extrinsics']['T_lidar_imu']

    assert parameters['common'] == {
        'lid_topic': '/points_raw_fastlio',
        'imu_topic': '/go2w/imu',
        'time_sync_en': False,
        'time_offset_lidar_to_imu': 0.0,
    }
    assert parameters['preprocess'] == {
        'lidar_type': 2,
        'scan_line': 16,
        'blind': 0.5,
        'timestamp_unit': 0,
        'scan_rate': 10,
    }
    # Timestamp-sorted XT16 points must not be index-stride filtered because
    # the stride aliases with the sensor firing/ring order.
    assert parameters['point_filter_num'] == 1
    assert parameters['mapping']['fov_degree'] == 360.0
    assert parameters['mapping']['det_range'] == 100.0
    assert parameters['mapping']['extrinsic_est_en'] is False
    assert parameters['mapping']['extrinsic_T'] == lidar_to_imu['translation']
    assert parameters['mapping']['extrinsic_R'] == lidar_to_imu['rotation_matrix']


def test_xt16_lidar_to_imu_is_composed_from_base_mounts():
    calibration = yaml.safe_load(
        (REPO_ROOT / 'config/sensor/go2w_xt16_calibration.yaml').read_text(
            encoding='utf-8'
        )
    )
    extrinsics = calibration['extrinsics']
    base_lidar = extrinsics['T_baselink_lidar']
    base_imu = extrinsics['T_baselink_imu']
    lidar_imu = extrinsics['T_lidar_imu']

    expected_translation = [
        lidar - imu
        for lidar, imu in zip(base_lidar['translation'], base_imu['translation'])
    ]
    assert all(
        math.isclose(actual, expected, abs_tol=1e-12)
        for actual, expected in zip(lidar_imu['translation'], expected_translation)
    )
    assert lidar_imu['rotation_matrix'] == base_lidar['rotation_matrix']


def test_xt16_urdf_mounts_match_calibration_and_existing_imu():
    calibration = yaml.safe_load(
        (REPO_ROOT / 'config/sensor/go2w_xt16_calibration.yaml').read_text(
            encoding='utf-8'
        )
    )
    root = ET.parse(
        REPO_ROOT / 'humble_ws/src/go2w_description/urdf/go2w_description.urdf'
    ).getroot()
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    lidar_joint = joints['hesai_lidar_joint']
    imu_joint = joints['imu_joint']

    assert lidar_joint.find('parent').attrib['link'] == 'base_link'
    assert lidar_joint.find('child').attrib['link'] == 'hesai_lidar'
    assert [float(value) for value in lidar_joint.find('origin').attrib['xyz'].split()] == (
        calibration['extrinsics']['T_baselink_lidar']['translation']
    )
    assert math.isclose(
        float(lidar_joint.find('origin').attrib['rpy'].split()[2]), math.pi / 2,
        abs_tol=1e-12,
    )
    assert [float(value) for value in imu_joint.find('origin').attrib['xyz'].split()] == (
        calibration['extrinsics']['T_baselink_imu']['translation']
    )


def test_offline_launch_keeps_mid360_default_and_gates_xt16_adapter():
    launch_text = (
        REPO_ROOT
        / 'humble_ws/src/fastlio_go2w_bringup/launch/offline_fastlio.launch.py'
    ).read_text(encoding='utf-8')

    assert 'default_value="mid360"' in launch_text
    assert '"mid360_go2w_accuracy_offline.yaml"' in launch_text
    assert '"xt16_go2w_accuracy_offline.yaml"' in launch_text
    assert 'if sensor == "xt16"' in launch_text
    assert 'package="fastlio_go2w_hesai"' in launch_text
