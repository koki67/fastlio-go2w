"""FAST-LIO + odom-adapter composition."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("fastlio_go2w_bringup")

    fastlio_config = os.path.join(pkg, "config", "mid360_go2w.yaml")
    rviz_config = os.path.join(pkg, "rviz", "fastlio.rviz")

    rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 with the FAST-LIO visualization config.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock when replaying from rosbag.",
    )
    time_sync_arg = DeclareLaunchArgument(
        "time_sync_en",
        default_value="false",
        description="Enable FAST-LIO time-sync option.",
    )

    fastlio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            fastlio_config,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "common.time_sync_en": LaunchConfiguration("time_sync_en"),
            },
        ],
    )

    odom_adapter_node = Node(
        package="fastlio_go2w_bringup",
        executable="fastlio_odom_adapter",
        name="fastlio_odom_adapter",
        output="screen",
        parameters=[
            {
                "odom_input_topic": "/Odometry",
                "odom_output_topic": "/odom",
                "base_frame": "base_link",
                "imu_frame": "livox_imu_frame",
                "camera_init_frame": "camera_init",
                "odom_frame": "odom",
                "sensor_frame_retry_sec": 1.0,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        time_sync_arg,
        fastlio_node,
        odom_adapter_node,
        rviz_node,
    ])
