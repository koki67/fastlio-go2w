"""TF-only verification launch for validating lidar mount transforms."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("fastlio_go2w_bringup")

    rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 with the check_tf config.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Pass-through for robot_state_publisher.",
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "robot_description.launch.py")
        ),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg, "rviz", "check_tf.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        robot_description,
        rviz,
    ])
