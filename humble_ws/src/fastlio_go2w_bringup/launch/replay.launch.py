"""Replay a raw bag with single- or experimental multi-LiDAR FAST-LIO."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fastlio_go2w_bringup.livox_replay import resolve_livox_replay_graph


_OFFLINE_PROFILES = {"baseline", "fused-high", "fused-matched"}


def _processing(context, package_share):
    profile = LaunchConfiguration("profile").perform(context)
    config_override = LaunchConfiguration("config").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    lidar_format = LaunchConfiguration("lidar_format").perform(context)
    actions = []

    if profile == "legacy":
        graph = resolve_livox_replay_graph(lidar_format)
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
        config_path = config_override or os.path.join(
            package_share, "config", "mid360_go2w.yaml"
        )
        launch_path = os.path.join(package_share, "launch", "bringup.launch.py")
        launch_arguments = {
            "with_sensors": "false",
            "use_sim_time": "true",
            "use_rviz": rviz,
            "config": config_path,
            "lid_topic_override": graph.fastlio_topic,
        }
    else:
        if profile not in _OFFLINE_PROFILES:
            choices = ", ".join(["legacy", *sorted(_OFFLINE_PROFILES)])
            raise RuntimeError(f"Unknown replay profile '{profile}': choose {choices}")
        launch_path = os.path.join(
            package_share, "launch", "offline_multilidar.launch.py"
        )
        launch_arguments = {
            "profile": profile,
            "use_rviz": rviz,
            "publish_debug_cloud": LaunchConfiguration("publish_debug_cloud").perform(
                context
            ),
            "lidar_format": lidar_format,
        }
        if config_override:
            launch_arguments["config"] = config_override

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments=launch_arguments.items(),
        )
    )
    return actions


def _play_bag(context):
    bag = LaunchConfiguration("bag").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    loop = LaunchConfiguration("loop").perform(context)
    profile = LaunchConfiguration("profile").perform(context)

    cmd = ["ros2", "bag", "play", bag, "--clock", "--rate", rate]
    if profile in _OFFLINE_PROFILES:
        cmd.extend(
            ["--topics", "/livox/lidar", "/livox/imu", "/points_raw"]
        )
    if loop == "true":
        cmd.append("--loop")

    return [ExecuteProcess(cmd=cmd, output="screen")]


def _delayed_play(context):
    delay_text = LaunchConfiguration("startup_delay").perform(context)
    try:
        delay = float(delay_text)
    except ValueError as error:
        raise RuntimeError("startup_delay must be a number") from error
    if delay < 0.0:
        raise RuntimeError("startup_delay must be non-negative")
    return [TimerAction(period=delay, actions=_play_bag(context))]


def generate_launch_description():
    package_share = get_package_share_directory("fastlio_go2w_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag",
                description="Path to raw bag directory to replay.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz2 with the FAST-LIO configuration.",
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
                "profile",
                default_value="legacy",
                description=(
                    "Replay profile: legacy, baseline, fused-high, or fused-matched."
                ),
            ),
            DeclareLaunchArgument(
                "config",
                default_value="",
                description="Optional FAST-LIO parameter YAML override.",
            ),
            DeclareLaunchArgument(
                "lidar_format",
                default_value="custom-msg",
                description="Resolved MID-360 input format: custom-msg or pointcloud2.",
            ),
            DeclareLaunchArgument(
                "publish_debug_cloud",
                default_value="false",
                description="Publish the source-labelled fused debug cloud.",
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="5.0",
                description="Seconds to wait for processing nodes before bag playback.",
            ),
            OpaqueFunction(
                function=_processing,
                args=[package_share],
            ),
            OpaqueFunction(function=_delayed_play),
        ]
    )
