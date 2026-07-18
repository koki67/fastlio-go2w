"""Offline FAST-LIO experiment for MID-360 and Pandar XT16 bags.

This launch file deliberately owns only processing nodes. Bag playback,
recording, readiness checks, and resource sampling are coordinated by
scripts/offline/run_multilidar_experiment.sh.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_PROFILES = {
    "baseline": None,
    "fused-high": {"mid_point_stride": 3, "hesai_firing_stride": 3},
    "fused-matched": {"mid_point_stride": 6, "hesai_firing_stride": 22},
}


def _processing_nodes(context, package_share):
    profile = LaunchConfiguration("profile").perform(context)
    if profile not in _PROFILES:
        choices = ", ".join(_PROFILES)
        raise RuntimeError(f"Unknown profile '{profile}'. Expected one of: {choices}")

    config_override = LaunchConfiguration("config").perform(context)
    if config_override:
        config_path = config_override
    elif profile == "baseline":
        config_path = os.path.join(
            package_share, "config", "mid360_go2w_accuracy_dense_false.yaml"
        )
    else:
        config_path = os.path.join(
            package_share, "config", "mid360_xt16_fused_accuracy_dense_false.yaml"
        )

    if not os.path.isfile(config_path):
        raise RuntimeError(
            f"FAST-LIO config does not exist: {config_path}. "
            "Rebuild the selected workspace overlay."
        )

    actions = []
    fusion_parameters = _PROFILES[profile]
    if fusion_parameters is not None:
        actions.append(
            Node(
                package="fastlio_go2w_fusion",
                executable="dual_lidar_fusion_node",
                name="dual_lidar_fusion",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "mid_topic": "/livox/lidar",
                        "hesai_topic": "/points_raw",
                        "output_topic": "/livox/lidar_fused",
                        "diagnostics_topic": "/fastlio_go2w_fusion/diagnostics",
                        "debug_topic": "/livox/lidar_fused_debug",
                        "mid_point_stride": fusion_parameters["mid_point_stride"],
                        "hesai_firing_stride": fusion_parameters["hesai_firing_stride"],
                        "min_range_m": 0.5,
                        "hesai_time_offset_sec": 0.0,
                        "max_pending_mid_frames": 32,
                        "publish_debug_cloud": ParameterValue(
                            LaunchConfiguration("publish_debug_cloud"),
                            value_type=bool,
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
                "use_sim_time": "true",
                "time_sync_en": "false",
                "config": config_path,
            }.items(),
        )
    )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("fastlio_go2w_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="baseline",
                description="Experiment profile: baseline, fused-high, or fused-matched.",
            ),
            DeclareLaunchArgument(
                "config",
                default_value="",
                description="Optional FAST-LIO YAML override for the selected profile.",
            ),
            DeclareLaunchArgument(
                "publish_debug_cloud",
                default_value="false",
                description="Publish the source-labelled fusion debug PointCloud2.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz2 with the FAST-LIO visualization config.",
            ),
            OpaqueFunction(
                function=_processing_nodes,
                args=[package_share],
            ),
        ]
    )
