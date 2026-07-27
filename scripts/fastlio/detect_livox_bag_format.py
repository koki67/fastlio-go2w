#!/usr/bin/env python3
"""Detect the supported /livox/lidar representation in rosbag2 metadata."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any, Mapping

import yaml


LIDAR_TOPIC = "/livox/lidar"
SUPPORTED_TYPES = {
    "livox_ros_driver2/msg/CustomMsg": "custom-msg",
    "sensor_msgs/msg/PointCloud2": "pointcloud2",
}


class DetectionError(ValueError):
    """Raised when metadata cannot establish one supported, nonempty input."""


@dataclass(frozen=True)
class LivoxBagFormat:
    format_name: str
    ros_type: str
    message_count: int


def detect_livox_bag_format(document: Any) -> LivoxBagFormat:
    """Return the unique supported Livox format described by parsed metadata."""
    if not isinstance(document, Mapping):
        raise DetectionError("metadata root must be a mapping")

    information = document.get("rosbag2_bagfile_information", document)
    if not isinstance(information, Mapping):
        raise DetectionError("rosbag2_bagfile_information must be a mapping")

    topics = information.get("topics_with_message_count")
    if not isinstance(topics, list):
        raise DetectionError("metadata has no topics_with_message_count list")

    counts_by_type: dict[str, int] = {}
    for index, entry in enumerate(topics):
        if not isinstance(entry, Mapping):
            raise DetectionError(f"topic entry {index} must be a mapping")
        topic_metadata = entry.get("topic_metadata")
        if not isinstance(topic_metadata, Mapping):
            raise DetectionError(f"topic entry {index} has no topic_metadata mapping")
        if topic_metadata.get("name") != LIDAR_TOPIC:
            continue

        ros_type = topic_metadata.get("type")
        if not isinstance(ros_type, str) or not ros_type:
            raise DetectionError(f"{LIDAR_TOPIC} has no valid ROS type")
        try:
            message_count = int(entry.get("message_count"))
        except (TypeError, ValueError) as exc:
            raise DetectionError(
                f"{LIDAR_TOPIC} has an invalid message_count"
            ) from exc
        if message_count < 0:
            raise DetectionError(f"{LIDAR_TOPIC} has a negative message_count")
        counts_by_type[ros_type] = counts_by_type.get(ros_type, 0) + message_count

    if not counts_by_type:
        raise DetectionError(f"metadata does not contain {LIDAR_TOPIC}")
    if len(counts_by_type) != 1:
        types = ", ".join(sorted(counts_by_type))
        raise DetectionError(
            f"{LIDAR_TOPIC} is recorded with multiple ROS types: {types}"
        )

    ros_type, message_count = next(iter(counts_by_type.items()))
    format_name = SUPPORTED_TYPES.get(ros_type)
    if format_name is None:
        supported = ", ".join(sorted(SUPPORTED_TYPES))
        raise DetectionError(
            f"unsupported ROS type for {LIDAR_TOPIC}: {ros_type}; "
            f"supported types: {supported}"
        )
    if message_count == 0:
        raise DetectionError(f"{LIDAR_TOPIC} has zero recorded messages")

    return LivoxBagFormat(format_name, ros_type, message_count)


def detect_metadata_file(path: Path) -> LivoxBagFormat:
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except FileNotFoundError as exc:
        raise DetectionError(f"metadata file not found: {path}") from exc
    except OSError as exc:
        raise DetectionError(f"could not read metadata file {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise DetectionError(f"could not parse metadata file {path}: {exc}") from exc
    return detect_livox_bag_format(document)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=f"Detect the supported ROS type recorded on {LIDAR_TOPIC}."
    )
    parser.add_argument("metadata", type=Path, help="path to rosbag2 metadata.yaml")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        detected = detect_metadata_file(args.metadata)
    except DetectionError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    print(detected.format_name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
