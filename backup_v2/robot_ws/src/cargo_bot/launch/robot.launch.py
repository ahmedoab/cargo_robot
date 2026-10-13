"""Starts everything that runs on the robot itself.

    ros2 launch cargo_bot robot.launch.py                  # base + payload + lidar
    ros2 launch cargo_bot robot.launch.py vision:=true     # + camera / line follower
    ros2 launch cargo_bot robot.launch.py lidar:=false payload:=false   # just the wheels

Command flow:  (teleop | Nav2 | line follower) -> /cmd_vel -> speed_limiter -> /cmd_vel_safe -> base_driver
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("cargo_bot")
    config = os.path.join(pkg, "config", "robot.yaml")
    laser_filter = os.path.join(pkg, "config", "laser_filter.yaml")
    xacro_file = os.path.join(pkg, "urdf", "cargo_bot.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    lidar = LaunchConfiguration("lidar")
    return LaunchDescription([
        DeclareLaunchArgument("lidar", default_value="true"),
        DeclareLaunchArgument("payload", default_value="true"),
        DeclareLaunchArgument("vision", default_value="false"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="cargo_bot", executable="base_driver", parameters=[config],
             remappings=[("cmd_vel", "cmd_vel_safe")], output="screen"),
        Node(package="cargo_bot", executable="speed_limiter", parameters=[config], output="screen"),
        Node(package="cargo_bot", executable="payload_node", parameters=[config], output="screen",
             condition=IfCondition(LaunchConfiguration("payload"))),
        Node(package="cargo_bot", executable="vision_node", parameters=[config], output="screen",
             condition=IfCondition(LaunchConfiguration("vision"))),

        Node(package="rplidar_ros", executable="rplidar_node", name="rplidar_node", output="screen",
             parameters=[{"serial_port": LaunchConfiguration("lidar_port"), "serial_baudrate": 115200,
                          "frame_id": "laser", "angle_compensate": True}],
             condition=IfCondition(lidar)),
        Node(package="laser_filters", executable="scan_to_scan_filter_chain", parameters=[laser_filter],
             condition=IfCondition(lidar)),
    ])
