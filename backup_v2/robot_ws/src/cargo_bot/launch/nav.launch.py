"""Autonomous navigation on a saved map: map_server + AMCL + Nav2. Run robot.launch.py (or sim.launch.py) first.

    ros2 launch cargo_bot nav.launch.py map:=$HOME/maps/lab.yaml                     # real robot
    ros2 launch cargo_bot nav.launch.py map:=$HOME/maps/sim.yaml use_sim_time:=true  # Gazebo simulation
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
    params = os.path.join(get_package_share_directory("cargo_bot"), "config", "nav2_params.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("map", description="full path to the map .yaml saved in week 6"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup), launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": params,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
        }.items()),
    ])
