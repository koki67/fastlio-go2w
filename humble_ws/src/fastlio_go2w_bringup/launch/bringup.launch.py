"""Top-level composition: robot description, sensors, and FAST-LIO."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    pkg = get_package_share_directory("fastlio_go2w_bringup")

    with_sensors_arg = DeclareLaunchArgument(
        "with_sensors",
        default_value="true",
        description="Start the Livox sensor nodes (false for bag replay).",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 for FAST-LIO and robot model views.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock from rosbag when replaying.",
    )
    time_sync_arg = DeclareLaunchArgument(
        "time_sync_en",
        default_value="false",
        description="Enable FAST-LIO time-sync mode.",
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "robot_description.launch.py")
        ),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "sensors.launch.py")),
        condition=IfCondition(LaunchConfiguration("with_sensors")),
    )

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "fastlio.launch.py")),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "time_sync_en": LaunchConfiguration("time_sync_en"),
        }.items(),
    )

    return LaunchDescription([
        with_sensors_arg,
        use_rviz_arg,
        use_sim_time_arg,
        time_sync_arg,
        robot_description,
        sensors,
        fastlio,
    ])
