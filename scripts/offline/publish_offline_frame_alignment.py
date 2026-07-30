#!/usr/bin/env python3
"""Publish the base-aligned odom-to-FAST-LIO map transform for offline RViz."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml


DEFAULT_CALIBRATION = (
    Path(__file__).resolve().parents[2]
    / "config"
    / "sensor"
    / "go2w_mid360_calibration.yaml"
)


class FrameAlignmentError(ValueError):
    """Raised when the offline display alignment is missing or malformed."""


@dataclass(frozen=True)
class FrameAlignment:
    fixed_frame_id: str
    map_frame_id: str
    translation_xyz: np.ndarray
    rotation_xyzw: np.ndarray


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise FrameAlignmentError(f"{label} must be a mapping")
    return value


def _frame_id(value: Any, label: str) -> str:
    normalized = str(value or "").strip().lstrip("/")
    if not normalized:
        raise FrameAlignmentError(f"{label} must not be empty")
    return normalized


def _finite_vector(value: Any, size: int, label: str) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=np.float64).reshape((-1,))
    except (TypeError, ValueError) as error:
        raise FrameAlignmentError(f"{label} must contain numeric values") from error
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise FrameAlignmentError(f"{label} must contain {size} finite values")
    return vector


def load_frame_alignment(calibration_path: Path | str) -> FrameAlignment:
    """Load the odom-to-camera_init transform used by the online adapter."""

    path = Path(calibration_path).expanduser().resolve()
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except FileNotFoundError as error:
        raise FrameAlignmentError(f"calibration not found: {path}") from error
    except yaml.YAMLError as error:
        raise FrameAlignmentError(f"invalid calibration YAML: {path}: {error}") from error

    root = _mapping(document, "calibration root")
    frames = _mapping(root.get("frames"), "calibration.frames")
    extrinsics = _mapping(root.get("extrinsics"), "calibration.extrinsics")
    base_to_imu = _mapping(
        extrinsics.get("T_baselink_imu"),
        "calibration.extrinsics.T_baselink_imu",
    )

    translation = _finite_vector(
        base_to_imu.get("translation"),
        3,
        "T_baselink_imu.translation",
    )
    rotation = _finite_vector(
        base_to_imu.get("rotation_quaternion"),
        4,
        "T_baselink_imu.rotation_quaternion",
    )
    norm = float(np.linalg.norm(rotation))
    if norm <= 1.0e-12:
        raise FrameAlignmentError("T_baselink_imu quaternion must be nonzero")
    rotation /= norm

    return FrameAlignment(
        fixed_frame_id=_frame_id(frames.get("odom"), "frames.odom"),
        map_frame_id=_frame_id(frames.get("camera_init"), "frames.camera_init"),
        translation_xyz=translation,
        rotation_xyzw=rotation,
    )


def alignment_record(alignment: FrameAlignment) -> dict[str, Any]:
    """Return a serializable description useful for validation and tests."""

    return {
        "parent_frame_id": alignment.fixed_frame_id,
        "child_frame_id": alignment.map_frame_id,
        "translation_xyz": alignment.translation_xyz.tolist(),
        "rotation_xyzw": alignment.rotation_xyzw.tolist(),
    }


def publish_alignment(alignment: FrameAlignment, repeat_period_s: float) -> int:
    """Keep the static transform available across simulated-time resets."""

    try:
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.clock import Clock, ClockType
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from tf2_ros import StaticTransformBroadcaster
    except ImportError as error:
        raise RuntimeError(
            "Frame alignment requires a sourced ROS 2 Humble environment"
        ) from error

    class AlignmentPublisher(Node):
        def __init__(self) -> None:
            super().__init__("fastlio_offline_frame_alignment")
            self._broadcaster = StaticTransformBroadcaster(self)
            self._transform = TransformStamped()
            self._transform.header.frame_id = alignment.fixed_frame_id
            self._transform.child_frame_id = alignment.map_frame_id
            translation = self._transform.transform.translation
            translation.x = float(alignment.translation_xyz[0])
            translation.y = float(alignment.translation_xyz[1])
            translation.z = float(alignment.translation_xyz[2])
            rotation = self._transform.transform.rotation
            rotation.x = float(alignment.rotation_xyzw[0])
            rotation.y = float(alignment.rotation_xyzw[1])
            rotation.z = float(alignment.rotation_xyzw[2])
            rotation.w = float(alignment.rotation_xyzw[3])
            self._sent_once = False
            steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._timer = self.create_timer(
                repeat_period_s, self._send, clock=steady_clock
            )
            self._send()

        def _send(self) -> None:
            self._transform.header.stamp = self.get_clock().now().to_msg()
            self._broadcaster.sendTransform(self._transform)
            if not self._sent_once:
                self.get_logger().info(
                    f"Publishing offline frame alignment "
                    f"{alignment.fixed_frame_id}->{alignment.map_frame_id}"
                )
                self._sent_once = True

    rclpy.init(args=None)
    node = AlignmentPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION,
        help="GO2-W MID-360 calibration YAML",
    )
    output = parser.add_mutually_exclusive_group()
    output.add_argument("--print-fixed-frame-id", action="store_true")
    output.add_argument("--print-map-frame-id", action="store_true")
    output.add_argument("--validate-only", action="store_true")
    parser.add_argument(
        "--repeat-period",
        type=float,
        default=1.0,
        help="wall-clock seconds between static-TF refreshes (default: 1.0)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if not math.isfinite(args.repeat_period) or args.repeat_period <= 0.0:
            raise FrameAlignmentError("--repeat-period must be finite and positive")
        alignment = load_frame_alignment(args.calibration)
        if args.print_fixed_frame_id:
            print(alignment.fixed_frame_id)
            return 0
        if args.print_map_frame_id:
            print(alignment.map_frame_id)
            return 0
        if args.validate_only:
            print(json.dumps(alignment_record(alignment), sort_keys=True))
            return 0
        return publish_alignment(alignment, args.repeat_period)
    except (FrameAlignmentError, OSError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
