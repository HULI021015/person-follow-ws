from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("wrist_frame", default_value="camera_link"),
        DeclareLaunchArgument(
            "optical_frame", default_value="camera_color_optical_frame"
        ),
        DeclareLaunchArgument("grasp_pose_topic", default_value="/grasp_pose"),
        DeclareLaunchArgument("print_period", default_value="1.0"),
        Node(
            package="graspgen_tasks",
            executable="coordinate_debug_node",
            name="coordinate_debug",
            output="screen",
            parameters=[{
                "base_frame": LaunchConfiguration("base_frame"),
                "wrist_frame": LaunchConfiguration("wrist_frame"),
                "optical_frame": LaunchConfiguration("optical_frame"),
                "grasp_pose_topic": LaunchConfiguration("grasp_pose_topic"),
                "print_period": LaunchConfiguration("print_period"),
            }],
        ),
    ])

