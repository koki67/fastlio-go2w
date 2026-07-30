#!/usr/bin/env python3

import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from publish_offline_frame_alignment import (  # noqa: E402
    FrameAlignmentError,
    alignment_record,
    load_frame_alignment,
    main,
)


def _write_calibration(path: Path) -> Path:
    document = {
        "frames": {"odom": "/odom", "camera_init": "/camera_init"},
        "extrinsics": {
            "T_baselink_imu": {
                "translation": [0.2, 0.03, 0.15],
                "rotation_quaternion": [0.0, 0.2, 0.0, 0.98],
            }
        },
    }
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def test_load_frame_alignment_normalizes_frames_and_quaternion(tmp_path):
    path = _write_calibration(tmp_path / "calibration.yaml")

    alignment = load_frame_alignment(path)

    assert alignment.fixed_frame_id == "odom"
    assert alignment.map_frame_id == "camera_init"
    np.testing.assert_allclose(alignment.translation_xyz, [0.2, 0.03, 0.15])
    assert np.linalg.norm(alignment.rotation_xyzw) == pytest.approx(1.0)
    assert alignment_record(alignment)["parent_frame_id"] == "odom"


def test_load_frame_alignment_rejects_invalid_transform(tmp_path):
    path = _write_calibration(tmp_path / "calibration.yaml")
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    document["extrinsics"]["T_baselink_imu"]["rotation_quaternion"] = [0, 0, 0, 0]
    path.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(FrameAlignmentError, match="quaternion must be nonzero"):
        load_frame_alignment(path)


def test_print_frame_ids_without_starting_ros(tmp_path, capsys):
    path = _write_calibration(tmp_path / "calibration.yaml")

    assert main(["--calibration", str(path), "--print-fixed-frame-id"]) == 0
    assert capsys.readouterr().out == "odom\n"
    assert main(["--calibration", str(path), "--print-map-frame-id"]) == 0
    assert capsys.readouterr().out == "camera_init\n"
