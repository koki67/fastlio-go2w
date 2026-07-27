from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from detect_livox_bag_format import DetectionError, detect_livox_bag_format


SCRIPT = Path(__file__).with_name("detect_livox_bag_format.py")


def document(*entries):
    return {
        "rosbag2_bagfile_information": {
            "topics_with_message_count": list(entries)
        }
    }


def topic(ros_type, count=10, name="/livox/lidar"):
    return {
        "topic_metadata": {"name": name, "type": ros_type},
        "message_count": count,
    }


@pytest.mark.parametrize(
    ("ros_type", "expected"),
    [
        ("livox_ros_driver2/msg/CustomMsg", "custom-msg"),
        ("sensor_msgs/msg/PointCloud2", "pointcloud2"),
    ],
)
def test_detects_supported_formats(ros_type, expected):
    detected = detect_livox_bag_format(document(topic(ros_type)))
    assert detected.format_name == expected
    assert detected.ros_type == ros_type
    assert detected.message_count == 10


def test_aggregates_duplicate_entries_of_the_same_type():
    detected = detect_livox_bag_format(
        document(
            topic("sensor_msgs/msg/PointCloud2", 4),
            topic("sensor_msgs/msg/PointCloud2", 6),
        )
    )
    assert detected.message_count == 10


@pytest.mark.parametrize(
    ("value", "message"),
    [
        (document(topic("sensor_msgs/msg/Imu", name="/livox/imu")), "does not contain"),
        (document(topic("sensor_msgs/msg/PointCloud2", 0)), "zero recorded"),
        (document(topic("example/msg/Cloud")), "unsupported ROS type"),
        ({"rosbag2_bagfile_information": {}}, "topics_with_message_count"),
    ],
)
def test_rejects_invalid_metadata(value, message):
    with pytest.raises(DetectionError, match=message):
        detect_livox_bag_format(value)


def test_rejects_multiple_types_for_same_topic():
    with pytest.raises(DetectionError, match="multiple ROS types"):
        detect_livox_bag_format(
            document(
                topic("livox_ros_driver2/msg/CustomMsg"),
                topic("sensor_msgs/msg/PointCloud2"),
            )
        )


def test_cli_prints_only_format_token(tmp_path):
    metadata = tmp_path / "metadata.yaml"
    metadata.write_text(
        yaml.safe_dump(document(topic("sensor_msgs/msg/PointCloud2"))),
        encoding="utf-8",
    )
    completed = subprocess.run(
        [sys.executable, str(SCRIPT), str(metadata)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0
    assert completed.stdout == "pointcloud2\n"
    assert completed.stderr == ""


def test_cli_rejects_malformed_yaml(tmp_path):
    metadata = tmp_path / "metadata.yaml"
    metadata.write_text("root: [unterminated\n", encoding="utf-8")
    completed = subprocess.run(
        [sys.executable, str(SCRIPT), str(metadata)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode != 0
    assert "could not parse metadata" in completed.stderr


def test_cli_rejects_missing_file(tmp_path):
    completed = subprocess.run(
        [sys.executable, str(SCRIPT), str(tmp_path / "missing.yaml")],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode != 0
    assert "metadata file not found" in completed.stderr
