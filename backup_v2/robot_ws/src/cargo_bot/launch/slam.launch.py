"""Mapping: slam_toolbox with our settings (uses /scan_filtered). Run robot.launch.py first.

    ros2 launch cargo_bot slam.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slam_launch = os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
    params = os.path.join(get_package_share_directory("cargo_bot"), "config", "slam.yaml")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch),
                                 launch_arguments={"slam_params_file": params, "use_sim_time": "false"}.items()),
    ])
