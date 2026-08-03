"""Contract tests for the raw MID-360/XT16 current-scan viewer."""

from pathlib import Path
import subprocess

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPT = SCRIPT_DIR / "view_raw_scans.sh"
RVIZ_CONFIG = SCRIPT_DIR / "rviz" / "raw_scans.rviz"


def make_bag(tmp_path, *, mid_type="sensor_msgs/msg/PointCloud2", include_xt=True):
    bag = tmp_path / "bag"
    bag.mkdir()
    topics = [
        {
            "topic_metadata": {
                "name": "/livox/lidar",
                "type": mid_type,
            },
            "message_count": 11,
        }
    ]
    if include_xt:
        topics.append(
            {
                "topic_metadata": {
                    "name": "/points_raw",
                    "type": "sensor_msgs/msg/PointCloud2",
                },
                "message_count": 12,
            }
        )
    document = {
        "rosbag2_bagfile_information": {
            "topics_with_message_count": topics,
        }
    }
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(document), encoding="utf-8"
    )
    return bag


def run(*args):
    return subprocess.run(
        ["bash", str(SCRIPT), *map(str, args)],
        check=False,
        capture_output=True,
        text=True,
    )


def pointcloud_displays():
    document = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))
    displays = document["Visualization Manager"]["Displays"]
    return [item for item in displays if item["Class"].endswith("/PointCloud2")]


def test_help_describes_current_scan_without_fast_lio():
    completed = run("--help")

    assert completed.returncode == 0
    assert "MID-360 only, XT16 only, or both" in completed.stdout
    assert "does not run FAST-LIO" in completed.stdout


def test_dry_run_validates_both_topics_and_limits_playback(tmp_path):
    completed = run(make_bag(tmp_path), "--dry-run", "--loop", "--rate", "2.0")

    assert completed.returncode == 0, completed.stderr
    assert "MID-360 /livox/lidar: 11 PointCloud2 messages" in completed.stdout
    assert "XT16 /points_raw: 12 PointCloud2 messages" in completed.stdout
    assert completed.stdout.count("static_transform_publisher") == 2
    assert "--frame-id base_link --child-frame-id livox_frame" in completed.stdout
    assert "--frame-id livox_frame --child-frame-id hesai_lidar" in completed.stdout
    assert "Fixed frame: base_link (level display grid)" in completed.stdout
    assert "rviz2" in completed.stdout
    assert "ros2 bag play" in completed.stdout
    assert "--topics /livox/lidar /points_raw" in completed.stdout
    assert "--loop" in completed.stdout


def test_rejects_bag_without_xt16(tmp_path):
    completed = run(make_bag(tmp_path, include_xt=False), "--dry-run")

    assert completed.returncode != 0
    assert "required raw-scan topic is missing: /points_raw" in completed.stderr


def test_rejects_old_mid360_custom_message_contract(tmp_path):
    completed = run(
        make_bag(tmp_path, mid_type="livox_ros_driver2/msg/CustomMsg"),
        "--dry-run",
    )

    assert completed.returncode != 0
    assert "current PointCloud2 recorder contract" in completed.stderr


def test_rviz_has_two_independent_non_accumulating_scan_displays():
    displays = pointcloud_displays()

    assert len(displays) == 2
    assert {item["Topic"]["Value"] for item in displays} == {
        "/livox/lidar",
        "/points_raw",
    }
    assert all(item["Decay Time"] == 0 for item in displays)
    assert all(item["Enabled"] is True for item in displays)
    assert all(item["Value"] is True for item in displays)
    assert all(item["Topic"]["Reliability Policy"] == "Best Effort" for item in displays)
    assert len({item["Color"] for item in displays}) == 2


def test_rviz_uses_level_base_frame_and_readable_display_names():
    document = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))
    manager = document["Visualization Manager"]

    assert manager["Global Options"]["Fixed Frame"] == "base_link"
    grids = [item for item in manager["Displays"] if item["Class"].endswith("/Grid")]
    assert len(grids) == 1
    assert grids[0]["Name"] == "Level grid (base_link XY)"
    assert grids[0]["Plane"] == "XY"
    names = {item["Name"] for item in pointcloud_displays()}
    assert names == {
        "MID-360 current scan (cyan)",
        "XT16 current scan (orange)",
    }
