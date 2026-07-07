from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from fastlio_go2w_bringup.math_utils import (
    pose_matrix_from_pose_msg,
    pose_msg_from_matrix,
    rebase_transform_fastlio_to_base,
)


def _pose(position, orientation):
    return SimpleNamespace(
        position=SimpleNamespace(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
        ),
        orientation=SimpleNamespace(
            x=float(orientation[0]),
            y=float(orientation[1]),
            z=float(orientation[2]),
            w=float(orientation[3]),
        ),
    )


def test_pose_roundtrip():
    pose = _pose((0.1, -0.2, 0.3), (0.0, 0.15883, 0.0, 0.987306))

    matrix = pose_matrix_from_pose_msg(pose)
    out = _pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    pose_msg_from_matrix(matrix, out)

    assert matrix.shape == (4, 4)
    assert out.position.x == 0.1
    assert out.position.y == -0.2
    assert out.position.z == 0.3
    assert np.isclose(out.orientation.w, 0.987306, atol=1e-6)


def test_rebase_transform_identity_base_to_imu():
    identity = np.eye(4)
    body = np.array(
        [
            [1.0, 0.0, 0.0, 0.3],
            [0.0, 1.0, 0.0, 0.4],
            [0.0, 0.0, 1.0, -0.2],
            [0.0, 0.0, 0.0, 1.0],
        ],
    )

    assert np.allclose(rebase_transform_fastlio_to_base(body, identity), body)


def test_rebase_transform_pitched_hand_case():
    # base->imu with target pitch and non-zero translation
    pitch = 0.319012
    cos_p = np.cos(pitch)
    sin_p = np.sin(pitch)
    base_to_imu = np.array(
        [
            [cos_p, 0.0, sin_p, 0.211],
            [0.0, 1.0, 0.0, 0.0],
            [-sin_p, 0.0, cos_p, 0.2008],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )

    body_matrix = np.array(
        [
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )

    expected = np.array(
        [
            [1.0, 0.0, 0.0, 0.949545746],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, -0.313628563],
            [0.0, 0.0, 0.0, 1.0],
        ],
    )

    assert np.allclose(
        rebase_transform_fastlio_to_base(body_matrix, base_to_imu),
        expected,
        atol=1e-6,
    )


def test_rebase_transform_shape_validation():
    with pytest.raises(ValueError):
        rebase_transform_fastlio_to_base(np.eye(3), np.eye(4))
