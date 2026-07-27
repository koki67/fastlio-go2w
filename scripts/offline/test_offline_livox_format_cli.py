from pathlib import Path
import subprocess

import yaml


SCRIPT = Path(__file__).with_name("run_fastlio_offline.sh")


def make_bag(tmp_path, ros_type="sensor_msgs/msg/PointCloud2"):
    bag = tmp_path / "bag"
    bag.mkdir()
    document = {
        "rosbag2_bagfile_information": {
            "topics_with_message_count": [
                {
                    "topic_metadata": {
                        "name": "/livox/lidar",
                        "type": ros_type,
                    },
                    "message_count": 1,
                },
                {
                    "topic_metadata": {
                        "name": "/livox/imu",
                        "type": "sensor_msgs/msg/Imu",
                    },
                    "message_count": 1,
                },
            ]
        }
    }
    (bag / "metadata.yaml").write_text(yaml.safe_dump(document), encoding="utf-8")
    return bag


def run(*args):
    return subprocess.run(
        ["bash", str(SCRIPT), *map(str, args)],
        check=False,
        capture_output=True,
        text=True,
    )


def test_help_documents_lidar_format():
    completed = run("--help")
    assert completed.returncode == 0
    assert "--lidar-format" in completed.stdout


def test_rejects_unknown_format_before_environment_preflight(tmp_path):
    completed = run(make_bag(tmp_path), "--lidar-format", "native")
    assert completed.returncode != 0
    assert "must be auto, custom-msg, or pointcloud2" in completed.stderr


def test_explicit_format_must_match_metadata(tmp_path):
    completed = run(make_bag(tmp_path), "--lidar-format", "custom-msg")
    assert completed.returncode != 0
    assert "conflicts with metadata format pointcloud2" in completed.stderr


def test_metadata_failure_is_fail_closed(tmp_path):
    completed = run(make_bag(tmp_path, "example/msg/Cloud"))
    assert completed.returncode != 0
    assert "unsupported ROS type" in completed.stderr
    assert "could not establish a supported Livox input format" in completed.stderr
