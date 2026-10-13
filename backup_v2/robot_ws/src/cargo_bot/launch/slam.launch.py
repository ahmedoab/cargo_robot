"""Mapping: slam_toolbox with our settings (uses /scan_filtered). Run robot.launch.py (or sim.launch.py) first.

    ros2 launch cargo_bot slam.launch.py                      # real robot
    ros2 launch cargo_bot slam.launch.py use_sim_time:=true   # Gazebo simulation
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    slam_launch = os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
    params = os.path.join(get_package_share_directory("cargo_bot"), "config", "slam.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch), launch_arguments={
            "slam_params_file": params,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items()),
    ])
