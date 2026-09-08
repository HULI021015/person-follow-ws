from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("graspgen_perception"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "engine_path",
            default_value=str(share / "config" / "yolov8n-seg.engine"),
        ),
        DeclareLaunchArgument(
            "labels_path",
            default_value=str(share / "config" / "coco.txt"),
        ),
        DeclareLaunchArgument("color_image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument(
            "aligned_depth_topic",
            default_value="/camera/depth/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/color/camera_info",
        ),
        DeclareLaunchArgument(
            "camera_frame",
            default_value="camera_color_optical_frame",
        ),
        DeclareLaunchArgument("show_gui", default_value="false"),
        Node(
            package="graspgen_perception",
            executable="yolov8_seg_node",
            name="graspgen_yolov8_seg",
            output="screen",
            parameters=[
                str(share / "config" / "perception.yaml"),
                {
                    "engine_path": LaunchConfiguration("engine_path"),
                    "labels_path": LaunchConfiguration("labels_path"),
                    "color_image_topic": LaunchConfiguration("color_image_topic"),
                    "aligned_depth_topic": LaunchConfiguration("aligned_depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "camera_frame": LaunchConfiguration("camera_frame"),
                    "show_gui": LaunchConfiguration("show_gui"),
                },
            ],
        ),
    ])
