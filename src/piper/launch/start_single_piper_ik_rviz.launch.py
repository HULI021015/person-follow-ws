from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_source(package, filename):
    return PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory(package)) / "launch" / filename)
    )


def generate_launch_description():
    can_port = DeclareLaunchArgument(
        "can_port",
        default_value="can0",
        description="Piper CAN interface. Run can_activate.sh before launching.",
    )
    auto_enable = DeclareLaunchArgument(
        "auto_enable", default_value="true"
    )
    piper = IncludeLaunchDescription(
        launch_source("piper", "start_single_piper_rviz.launch.py"),
        launch_arguments={
            "can_port": LaunchConfiguration("can_port"),
            "auto_enable": LaunchConfiguration("auto_enable"),
        }.items(),
    )
    ik = IncludeLaunchDescription(
        launch_source("piper_kinematics_ros2", "ik.launch.py")
    )
    return LaunchDescription([can_port, auto_enable, piper, ik])
