from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(
        get_package_share_directory("piper_description")
    )
    model = description_share / "urdf" / "piper_description.urdf"
    rviz_config = description_share / "rviz" / "piper_ctrl.rviz"
    robot_description = model.read_text(encoding="utf-8")

    return LaunchDescription([
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("auto_enable", default_value="true"),
        DeclareLaunchArgument("gripper_exist", default_value="true"),
        DeclareLaunchArgument("gripper_val_mutiple", default_value="1"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="piper_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="piper",
            executable="piper_single_ctrl",
            name="piper_ctrl_single_node",
            output="screen",
            parameters=[{
                "can_port": LaunchConfiguration("can_port"),
                "auto_enable": LaunchConfiguration("auto_enable"),
                "gripper_exist": LaunchConfiguration("gripper_exist"),
                "gripper_val_mutiple": LaunchConfiguration(
                    "gripper_val_mutiple"
                ),
            }],
            remappings=[
                ("joint_ctrl_single", "/joint_ctrl_cmd"),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="piper_rviz2",
            output="screen",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("start_rviz")),
        ),
    ])
