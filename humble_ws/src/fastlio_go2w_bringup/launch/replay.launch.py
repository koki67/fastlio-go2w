"""Replay FAST-LIO from raw bag with FAST-LIO configuration in simulation time."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _play_bag(context, *_args, **_kwargs):
    bag = LaunchConfiguration("bag").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    loop = LaunchConfiguration("loop").perform(context)

    cmd = ["ros2", "bag", "play", bag, "--clock", "--rate", rate]
    if loop == "true":
        cmd.append("--loop")

    return [ExecuteProcess(cmd=cmd, output="screen")]


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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "bringup.launch.py")
            ),
            launch_arguments={
                "with_sensors": "false",
                "use_sim_time": "true",
                "use_rviz": LaunchConfiguration("rviz"),
                "config": LaunchConfiguration("config"),
            }.items(),
        ),
        OpaqueFunction(function=_play_bag),
    ])
