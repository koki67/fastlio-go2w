#!/usr/bin/env python3
"""Extract the final Livox adapter DiagnosticArray from an offline result bag."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Sequence


STATUS_NAME = "fastlio_go2w_livox/pointcloud_adapter"
DEFAULT_TOPIC = "/fastlio_go2w_livox/diagnostics"
INTEGER_KEYS = {
    "received_frames",
    "converted_frames",
    "critical_drops",
    "reordered_frames",
    "quantization_clamped_frames",
    "quantization_clamped_points",
    "dropped_invalid_header",
    "dropped_header_regression",
    "dropped_schema",
    "dropped_layout",
    "dropped_nonfinite_coordinate",
    "dropped_nonfinite_intensity",
    "dropped_nonfinite_timestamp",
    "dropped_intensity_out_of_range",
    "dropped_intensity_nonintegral",
    "dropped_negative_offset",
    "dropped_offset_too_large",
    "dropped_too_few_points",
}
FLOAT_KEYS = {"latest_scan_width_sec"}
DROP_KEYS = {key for key in INTEGER_KEYS if key.startswith("dropped_")}


def _typed_value(key: str, value: str) -> Any:
    if key in INTEGER_KEYS:
        return int(value)
    if key in FLOAT_KEYS:
        return float(value)
    return value


def diagnostic_document(
    message: Any,
    *,
    topic: str,
    message_count: int,
    bag_timestamp_ns: int,
) -> dict[str, Any]:
    statuses = [status for status in message.status if status.name == STATUS_NAME]
    if len(statuses) != 1:
        raise ValueError(
            "final DiagnosticArray must contain exactly one Livox adapter status"
        )
    status = statuses[0]
    raw_values: dict[str, str] = {}
    for item in status.values:
        if item.key in raw_values:
            raise ValueError(f"duplicate diagnostic key: {item.key}")
        raw_values[item.key] = item.value
    required = INTEGER_KEYS | FLOAT_KEYS | {"latest_drop_reason"}
    missing = sorted(required - raw_values.keys())
    if missing:
        raise ValueError("diagnostics missing keys: " + ", ".join(missing))
    try:
        values = {key: _typed_value(key, value) for key, value in raw_values.items()}
    except ValueError as exc:
        raise ValueError(f"diagnostics contain an invalid numeric value: {exc}") from exc

    if any(values[key] < 0 for key in INTEGER_KEYS):
        raise ValueError("adapter counters must be non-negative")
    dropped = sum(values[key] for key in DROP_KEYS)
    if values["critical_drops"] != dropped:
        raise ValueError(
            "critical drop total mismatch: "
            f"reported={values['critical_drops']}, summed={dropped}"
        )
    received = values["received_frames"]
    converted = values["converted_frames"]
    if received != converted + dropped:
        raise ValueError(
            "adapter accounting mismatch: "
            f"received={received}, converted={converted}, dropped={dropped}"
        )
    if values["reordered_frames"] > converted:
        raise ValueError("reordered_frames cannot exceed converted_frames")
    if values["quantization_clamped_frames"] > converted:
        raise ValueError("quantization_clamped_frames cannot exceed converted_frames")
    if values["latest_scan_width_sec"] < 0:
        raise ValueError("latest_scan_width_sec must be non-negative")

    level = (
        status.level[0]
        if isinstance(status.level, (bytes, bytearray))
        else int(status.level)
    )
    return {
        "schema_version": 1,
        "topic": topic,
        "message_count": message_count,
        "last_bag_timestamp_ns": bag_timestamp_ns,
        "last_header_stamp_ns": (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        ),
        "status": {
            "name": status.name,
            "level": level,
            "message": status.message,
            "hardware_id": status.hardware_id,
        },
        "values": values,
    }


def extract(result_bag: Path, topic: str) -> dict[str, Any]:
    import rosbag2_py
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    metadata_path = result_bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    information = metadata.get("rosbag2_bagfile_information", metadata)
    storage_id = information["storage_identifier"]

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(result_bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(f"diagnostics topic is absent from result bag: {topic}")
    if topic_types[topic] != "diagnostic_msgs/msg/DiagnosticArray":
        raise ValueError(f"unexpected diagnostics type {topic_types[topic]!r} on {topic}")
    message_type = get_message(topic_types[topic])
    final_message = None
    final_timestamp = 0
    count = 0
    while reader.has_next():
        current_topic, serialized, timestamp = reader.read_next()
        if current_topic != topic:
            continue
        final_message = deserialize_message(serialized, message_type)
        final_timestamp = timestamp
        count += 1
    if final_message is None:
        raise ValueError(f"diagnostics topic contains no messages: {topic}")
    return diagnostic_document(
        final_message,
        topic=topic,
        message_count=count,
        bag_timestamp_ns=final_timestamp,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        document = extract(args.result_bag.expanduser().resolve(), args.topic)
    except (FileNotFoundError, KeyError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(args.output)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
