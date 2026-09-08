from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory("piper_kinematics_ros2"))
        / "config"
        / "ik.yaml"
    )
    return LaunchDescription([
        Node(
            package="piper_kinematics_ros2",
            executable="piper_ik_node",
            name="piper_ik",
            output="screen",
            parameters=[str(config)],
        )
    ])
