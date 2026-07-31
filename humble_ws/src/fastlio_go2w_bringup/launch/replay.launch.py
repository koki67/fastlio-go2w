"""Replay FAST-LIO from raw bag with FAST-LIO configuration in simulation time."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fastlio_go2w_bringup.livox_replay import resolve_livox_replay_graph


def _play_bag(context, *_args, **_kwargs):
    bag = LaunchConfiguration("bag").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    loop = LaunchConfiguration("loop").perform(context)

    cmd = ["ros2", "bag", "play", bag, "--clock", "--rate", rate]
    if loop == "true":
        cmd.append("--loop")

    return [ExecuteProcess(cmd=cmd, output="screen")]


def _processing_graph(context, package_share):
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
                "use_sim_time": "true",
                "use_rviz": LaunchConfiguration("rviz"),
                "config": LaunchConfiguration("config"),
                "lid_topic_override": graph.fastlio_topic,
            }.items(),
        )
    )
    return actions


def generate_launch_description():
    pkg = get_package_share_directory("fastlio_go2w_bringup")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag",
            description="Path to raw bag directory to replay.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Launch RViz2 with fastlio configuration.",
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="1.0",
            description="Playback speed multiplier.",
        ),
        DeclareLaunchArgument(
            "loop",
            default_value="false",
            description="Loop bag playback.",
        ),
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(pkg, "config", "mid360_go2w.yaml"),
            description="FAST-LIO parameter YAML file.",
        ),
        DeclareLaunchArgument(
            "lidar_format",
            default_value="custom-msg",
            description="Resolved Livox input format: custom-msg or pointcloud2.",
        ),
        OpaqueFunction(function=_processing_graph, args=[pkg]),
        OpaqueFunction(function=_play_bag),
    ])
