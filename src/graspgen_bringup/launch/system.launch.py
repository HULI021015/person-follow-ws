from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def package_launch(package, *parts):
    return PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory(package)).joinpath(*parts))
    )


def check_can_interface(context, *args, **kwargs):
    skip_check = LaunchConfiguration("skip_can_check").perform(context).lower()
    if skip_check in ("1", "true", "yes", "on"):
        return []

    can_port = LaunchConfiguration("can_port").perform(context)
    command = ["ip", "-details", "link", "show", can_port]
    result = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"CAN interface '{can_port}' was not found. "
            "Check the interface name with: ip link show type can. "
            f"If your adapter should be '{can_port}', activate it with: "
            f"cd ~/person_follow_ws && bash scripts/can_activate.sh {can_port} 1000000"
        )

    first_line = result.stdout.splitlines()[0] if result.stdout else ""
    if "UP" not in first_line:
        raise RuntimeError(
            f"CAN interface '{can_port}' exists but is not UP. "
            f"Run: cd ~/person_follow_ws && bash scripts/can_activate.sh {can_port} 1000000"
        )

    if "bitrate 1000000" not in result.stdout:
        raise RuntimeError(
            f"CAN interface '{can_port}' is not configured at 1000000 bitrate. "
            f"Run: cd ~/person_follow_ws && bash scripts/can_activate.sh {can_port} 1000000"
        )

    print(
        f"[graspgen_bringup] CAN preflight OK: {can_port} is UP at 1000000 bitrate"
    )
    return []


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument(
            "skip_can_check",
            default_value="false",
            description=(
                "Set true only for simulation/debug runs without a real Piper CAN interface."
            ),
        ),
    ]

    camera = IncludeLaunchDescription(
        package_launch(
            "graspgen_bringup", "launch", "dabai_dcl_grasp.launch.py"
        )
    )
    piper = IncludeLaunchDescription(
        package_launch("piper", "launch", "start_single_piper_rviz.launch.py"),
        launch_arguments={
            "can_port": LaunchConfiguration("can_port"),
            "auto_enable": "true",
            "start_rviz": LaunchConfiguration("start_rviz"),
        }.items(),
    )
    perception = IncludeLaunchDescription(
        package_launch(
            "graspgen_perception", "launch", "yolov8.launch.py"
        )
    )
    ik = IncludeLaunchDescription(
        package_launch(
            "piper_kinematics_ros2", "launch", "ik.launch.py"
        )
    )
    task = IncludeLaunchDescription(
        package_launch("graspgen_tasks", "launch", "task.launch.py")
    )
    return LaunchDescription(
        arguments + [OpaqueFunction(function=check_can_interface), camera, piper, perception, ik, task]
    )
