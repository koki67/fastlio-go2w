"""Headless offline FAST-LIO processing for recorded MID-360 bags.

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

from fastlio_go2w_bringup.livox_replay import resolve_livox_replay_graph


def _processing_nodes(context, package_share):
    config_override = LaunchConfiguration("config").perform(context)
    config_path = config_override or os.path.join(
        package_share, "config", "mid360_go2w_accuracy_offline.yaml"
    )

    if not os.path.isfile(config_path):
        raise RuntimeError(
            f"FAST-LIO config does not exist: {config_path}. "
            "Rebuild the selected workspace overlay."
        )

    graph = resolve_livox_replay_graph(
        LaunchConfiguration("lidar_format").perform(context)
    )
    actions = []
    if graph.adapter_enabled:
        actions.append(
            Node(
                package="fastlio_go2w_livox",
                executable="livox_pointcloud_adapter",
                name="livox_pointcloud_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": graph.input_topic,
                        "output_topic": graph.fastlio_topic,
                        "diagnostics_topic": graph.diagnostics_topic,
                    }
                ],
            )
        )
    actions.append(
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
                "lid_topic_override": graph.fastlio_topic,
            }.items(),
        )
    )
    return actions


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
                "lidar_format",
                default_value="custom-msg",
                description="Resolved Livox input format: custom-msg or pointcloud2.",
            ),
            OpaqueFunction(
                function=_processing_nodes,
                args=[package_share],
            ),
        ]
    )
