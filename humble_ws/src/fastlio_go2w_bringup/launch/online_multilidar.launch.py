"""Online-safe MID-360 + Pandar XT16 pre-LIO fusion and FAST-LIO graph."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from fastlio_go2w_bringup.fusion_profiles import (
    resolve_online_fusion_profile,
    validate_missing_sensor_policy,
)


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(f"{name} must be true or false, got '{value}'")


def _online_nodes(context, package_share):
    profile = LaunchConfiguration("profile").perform(context)
    policy = LaunchConfiguration("missing_sensor_policy").perform(context)
    try:
        strides = resolve_online_fusion_profile(profile)
        validate_missing_sensor_policy(policy)
    except ValueError as error:
        raise RuntimeError(str(error)) from error

    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    actions = []
    if _as_bool(context, "with_sensors"):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, "launch", "sensors.launch.py")
                )
            )
        )
        hesai_share = get_package_share_directory("hesai_lidar")
        actions.append(
            Node(
                package="hesai_lidar",
                executable="hesai_lidar_node",
                name="hesai_node",
                output="screen",
                parameters=[
                    {
                        "pcap_file": "",
                        "server_ip": LaunchConfiguration("hesai_server_ip"),
                        "lidar_recv_port": 2368,
                        "gps_port": 10110,
                        "start_angle": 0.0,
                        "lidar_type": "PandarXT-16",
                        "frame_id": "hesai_lidar",
                        "pcldata_type": 0,
                        "publish_type": "points",
                        "timestamp_type": "realtime",
                        "data_type": "",
                        "lidar_correction_file": os.path.join(
                            hesai_share, "config", "PandarXT-16.csv"
                        ),
                        "multicast_ip": "",
                        "coordinate_correction_flag": False,
                        "fixed_frame": "",
                        "target_frame": "",
                    }
                ],
            )
        )

    actions.append(
        Node(
            package="fastlio_go2w_fusion",
            executable="dual_lidar_fusion_node",
            name="dual_lidar_fusion",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "online_watchdog_enabled": True,
                    "fusion_profile": profile,
                    "missing_sensor_policy": policy,
                    "mid_topic": "/livox/lidar",
                    "hesai_topic": "/points_raw",
                    "output_topic": "/livox/lidar_fused",
                    "diagnostics_topic": "/fastlio_go2w_fusion/diagnostics",
                    "debug_topic": "/livox/lidar_fused_debug",
                    "mid_point_stride": strides["mid_point_stride"],
                    "hesai_firing_stride": strides["hesai_firing_stride"],
                    "min_range_m": 0.5,
                    "max_pending_mid_frames": 32,
                    "source_stale_timeout_sec": ParameterValue(
                        LaunchConfiguration("source_stale_timeout_sec"),
                        value_type=float,
                    ),
                    "startup_grace_sec": ParameterValue(
                        LaunchConfiguration("startup_grace_sec"),
                        value_type=float,
                    ),
                    "diagnostics_period_sec": 0.1,
                    "pending_flush_wall_timeout_sec": 0.0,
                    "publish_debug_cloud": ParameterValue(
                        LaunchConfiguration("publish_debug_cloud"), value_type=bool
                    ),
                    "hesai_to_livox.translation": [
                        -0.018602675,
                        0.0,
                        -0.095450199,
                    ],
                    "hesai_to_livox.rotation_xyzw": [
                        -0.112310121,
                        -0.112310121,
                        0.698130673,
                        0.698130673,
                    ],
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
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "time_sync_en": "false",
                "config": os.path.join(
                    package_share, "config", "mid360_xt16_fused.yaml"
                ),
                "lid_topic_override": "/livox/lidar_fused",
            }.items(),
        )
    )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("fastlio_go2w_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="fused-matched"),
            DeclareLaunchArgument(
                "missing_sensor_policy", default_value="strict"
            ),
            DeclareLaunchArgument("with_sensors", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("publish_debug_cloud", default_value="false"),
            DeclareLaunchArgument("source_stale_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("startup_grace_sec", default_value="2.0"),
            DeclareLaunchArgument("hesai_server_ip", default_value="192.168.123.20"),
            OpaqueFunction(function=_online_nodes, args=[package_share]),
        ]
    )
