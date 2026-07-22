"""Headless offline FAST-LIO processing for recorded MID-360 or XT16 bags.

This launch file owns only the processing nodes. Bag playback, output
recording, readiness checks, and resource sampling are coordinated by
scripts/offline/run_fastlio_offline.sh.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _processing_nodes(context, package_share):
    sensor = LaunchConfiguration("sensor").perform(context)
    if sensor not in {"mid360", "xt16"}:
        raise RuntimeError(
            f"Unsupported sensor profile {sensor!r}; expected 'mid360' or 'xt16'."
        )

    config_override = LaunchConfiguration("config").perform(context)
    default_config = (
        "mid360_go2w_accuracy_offline.yaml"
        if sensor == "mid360"
        else "xt16_go2w_accuracy_offline.yaml"
    )
    config_path = config_override or os.path.join(package_share, "config", default_config)

    if not os.path.isfile(config_path):
        raise RuntimeError(
            f"FAST-LIO config does not exist: {config_path}. "
            "Rebuild the selected workspace overlay."
        )

    nodes = []
    if sensor == "xt16":
        nodes.append(
            Node(
                package="fastlio_go2w_hesai",
                executable="hesai_pointcloud_adapter",
                name="hesai_pointcloud_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/points_raw",
                        "output_topic": "/points_raw_fastlio",
                        "diagnostics_topic": "/fastlio_go2w_hesai/diagnostics",
                        "lidar_time_offset_sec": LaunchConfiguration(
                            "lidar_time_offset_sec"
                        ),
                    }
                ],
            )
        )

    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, "launch", "bringup.launch.py")
            ),
            launch_arguments={
                "with_sensors": "false",
                "use_rviz": "false",
                "use_sim_time": "true",
                "time_sync_en": "false",
                "config": config_path,
                "imu_frame": "imu" if sensor == "xt16" else "livox_imu_frame",
            }.items(),
        )
    )
    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("fastlio_go2w_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="",
                description="Optional headless FAST-LIO YAML override.",
            ),
            DeclareLaunchArgument(
                "sensor",
                default_value="mid360",
                description="Offline input profile: mid360 or xt16.",
            ),
            DeclareLaunchArgument(
                "lidar_time_offset_sec",
                default_value="0.0",
                description=(
                    "Signed XT16 LiDAR header offset in seconds; ignored for mid360."
                ),
            ),
            OpaqueFunction(
                function=_processing_nodes,
                args=[package_share],
            ),
        ]
    )
