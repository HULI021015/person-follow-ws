from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory("graspgen_tasks"))
        / "config"
        / "task.yaml"
    )
    return LaunchDescription([
        Node(
            package="graspgen_tasks",
            executable="grasp_task_node",
            name="grasp_task",
            output="screen",
            parameters=[str(config)],
        )
    ])
