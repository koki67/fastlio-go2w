from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def _normalize_quat(quat: Iterable[float]) -> np.ndarray:
    q = np.asarray(list(quat), dtype=float)
    norm = np.linalg.norm(q)
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("Invalid quaternion (zero-length)")
    return q / norm


def pose_matrix_from_pose_msg(pose):
    tx = float(pose.position.x)
    ty = float(pose.position.y)
    tz = float(pose.position.z)
    qx, qy, qz, qw = _normalize_quat(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    )

    xy2 = qx * qy
    xz2 = qx * qz
    yz2 = qy * qz
    wx2 = qw * qx
    wy2 = qw * qy
    wz2 = qw * qz

    xx2 = qx * qx
    yy2 = qy * qy
    zz2 = qz * qz

    r00 = 1.0 - 2.0 * (yy2 + zz2)
    r01 = 2.0 * (xy2 - wz2)
    r02 = 2.0 * (xz2 + wy2)

    r10 = 2.0 * (xy2 + wz2)
    r11 = 1.0 - 2.0 * (xx2 + zz2)
    r12 = 2.0 * (yz2 - wx2)

    r20 = 2.0 * (xz2 - wy2)
    r21 = 2.0 * (yz2 + wx2)
    r22 = 1.0 - 2.0 * (xx2 + yy2)

    matrix = np.eye(4, dtype=float)
    matrix[0, 0] = r00
    matrix[0, 1] = r01
    matrix[0, 2] = r02
    matrix[1, 0] = r10
    matrix[1, 1] = r11
    matrix[1, 2] = r12
    matrix[2, 0] = r20
    matrix[2, 1] = r21
    matrix[2, 2] = r22

    matrix[0, 3] = tx
    matrix[1, 3] = ty
    matrix[2, 3] = tz
    return matrix


def pose_msg_from_matrix(matrix: np.ndarray, pose):
    pose.position.x = float(matrix[0, 3])
    pose.position.y = float(matrix[1, 3])
    pose.position.z = float(matrix[2, 3])

    r = matrix[:3, :3]
    tr = float(np.trace(r))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    else:
        if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
            qw = (r[2, 1] - r[1, 2]) / s
        elif r[1, 1] > r[2, 2]:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
            qw = (r[0, 2] - r[2, 0]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s
            qw = (r[1, 0] - r[0, 1]) / s

    q = _normalize_quat([qx, qy, qz, qw])
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])
    return pose


def rebase_transform_fastlio_to_base(odom_body: np.ndarray, base_to_imu: np.ndarray) -> np.ndarray:
    if odom_body.shape != (4, 4) or base_to_imu.shape != (4, 4):
        raise ValueError("Inputs must be 4x4 homogeneous matrices.")
    return base_to_imu @ odom_body @ np.linalg.inv(base_to_imu)
