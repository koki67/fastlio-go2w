"""Robot description publisher for visual/model TF checks."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = (
        Path(get_package_share_directory("go2w_description"))
        / "urdf"
        / "go2w_description.urdf"
    )
    robot_description = urdf_path.read_text(encoding="utf-8")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Pass through use_sim_time to robot_state_publisher.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="go2w_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="go2w_joint_state_publisher",
            output="screen",
        ),
    ])
