"""Gazebo simulation of the cargo robot (replaces robot.launch.py when there is no real robot).

    ros2 launch cargo_bot sim.launch.py                 # Gazebo window + robot in the test area
    ros2 launch cargo_bot sim.launch.py headless:=true  # no Gazebo window (faster)

Then, exactly as on the real robot:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 launch cargo_bot slam.launch.py use_sim_time:=true
    ros2 launch cargo_bot nav.launch.py map:=$HOME/maps/sim.yaml use_sim_time:=true

Simulated: wheels + odometry (Gazebo DiffDrive, in place of base_driver), lidar (/scan), camera
(/camera/image_raw). Not simulated: load cells (no /payload_weight, so speed_limiter runs at full speed),
E-stop, encoders/GPIO.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("cargo_bot")
    world = LaunchConfiguration("world")
    xacro_file = os.path.join(pkg, "urdf", "cargo_bot.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", xacro_file, " sim:=true"]), value_type=str)
    gz_sim = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    sim_time = {"use_sim_time": True}

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=os.path.join(pkg, "worlds", "test_area.sdf")),
        DeclareLaunchArgument("headless", default_value="false"),
        # lets Gazebo find package://cargo_bot/meshes/... from the URDF
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(pkg)),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_sim),
                                 launch_arguments={"gz_args": ["-r ", world]}.items(),
                                 condition=UnlessCondition(LaunchConfiguration("headless"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_sim),
                                 launch_arguments={"gz_args": ["-r -s ", world]}.items(),
                                 condition=IfCondition(LaunchConfiguration("headless"))),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}, sim_time]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "cargo_bot", "-z", "0.01"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(pkg, "config", "gz_bridge.yaml")}, sim_time]),

        # same processing chain as the real robot
        Node(package="cargo_bot", executable="speed_limiter", output="screen",
             parameters=[os.path.join(pkg, "config", "robot.yaml"), sim_time]),
        Node(package="laser_filters", executable="scan_to_scan_filter_chain",
             parameters=[os.path.join(pkg, "config", "laser_filter.yaml"), sim_time]),
    ])
