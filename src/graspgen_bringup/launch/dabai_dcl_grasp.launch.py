from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path


def generate_launch_description():
    """Dabai DCL profile migrated from the working ROS1 deployment.

    The ROS1 setup used 640x480 color and 640x400 depth, but disabled D2C
    registration and frame synchronization. That made mask pixel coordinates
    unsafe for depth access. ROS2 keeps the proven stream profiles while
    enabling registration and synchronization.
    """
    launch_file = (
        Path(get_package_share_directory("orbbec_camera"))
        / "launch"
        / "dabai_dcl.py"
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments={
            "camera_name": "camera",
            "color_width": "640",
            "color_height": "480",
            "color_fps": "15",
            "color_format": "MJPG",
            "depth_width": "640",
            "depth_height": "400",
            "depth_fps": "15",
            "depth_format": "Y12",
            "enable_ir": "false",
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
            "depth_registration": "true",
            "align_mode": "SW",
            "enable_frame_sync": "false",
            "sync_mode": "",
            "enable_ir_auto_exposure": "false",
            "enable_ir_long_exposure": "false",
            "enable_accel": "false",
            "enable_gyro": "false",
            "enable_sync_output_accel_gyro": "false",
            "publish_tf": "true",
        }.items(),
    )
    return LaunchDescription([camera])



