import os
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _netsdk_library_path():
    """Locate the native libraries installed by Dahua's Python wheel."""
    spec = importlib.util.find_spec("NetSDK")
    if spec is None or not spec.submodule_search_locations:
        return ""
    candidate = Path(next(iter(spec.submodule_search_locations))) / "Libs" / "linux64"
    return str(candidate) if candidate.is_dir() else ""


def generate_launch_description():
    follower_share = get_package_share_directory("person_follower")
    chassis_share = get_package_share_directory("vehicle_bringup")
    sdk_library_path = _netsdk_library_path()
    environment_actions = []
    if sdk_library_path:
        current = os.environ.get("LD_LIBRARY_PATH", "")
        value = sdk_library_path if not current else f"{sdk_library_path}:{current}"
        environment_actions.append(SetEnvironmentVariable("LD_LIBRARY_PATH", value))

    return LaunchDescription(environment_actions + [
        DeclareLaunchArgument(
            "follower_params",
            default_value=os.path.join(follower_share, "config", "follower.yaml"),
        ),
        DeclareLaunchArgument(
            "chassis_params",
            default_value=os.path.join(chassis_share, "config", "chassis.yaml"),
        ),
        DeclareLaunchArgument("start_chassis", default_value="true"),
        DeclareLaunchArgument("show_window", default_value="false"),
        Node(
            package="vehicle_bringup",
            executable="chassis_node",
            name="chassis_node",
            output="screen",
            parameters=[LaunchConfiguration("chassis_params")],
            condition=IfCondition(LaunchConfiguration("start_chassis")),
        ),
        Node(
            package="person_follower",
            executable="person_follower_node",
            name="person_follower_node",
            output="screen",
            parameters=[
                LaunchConfiguration("follower_params"),
                {
                    "show_window": ParameterValue(
                        LaunchConfiguration("show_window"), value_type=bool
                    )
                },
            ],
        ),
    ])
