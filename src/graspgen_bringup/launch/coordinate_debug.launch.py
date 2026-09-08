from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    debug_launch = (
        Path(get_package_share_directory("graspgen_tasks"))
        / "launch"
        / "coordinate_debug.launch.py"
    )
    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("wrist_frame", default_value="camera_link"),
        DeclareLaunchArgument(
            "optical_frame", default_value="camera_color_optical_frame"
        ),
        DeclareLaunchArgument("grasp_pose_topic", default_value="/grasp_pose"),
        DeclareLaunchArgument("print_period", default_value="1.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(debug_launch)),
            launch_arguments={
                "base_frame": LaunchConfiguration("base_frame"),
                "wrist_frame": LaunchConfiguration("wrist_frame"),
                "optical_frame": LaunchConfiguration("optical_frame"),
                "grasp_pose_topic": LaunchConfiguration("grasp_pose_topic"),
                "print_period": LaunchConfiguration("print_period"),
            }.items(),
        ),
    ])

