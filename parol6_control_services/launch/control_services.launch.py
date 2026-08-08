"""Starts parol6_control_services' nodes: the Trigger-service node
(home/gripper) and call_method_node's generic CallMethod pass-through to
any RobotClient method.

Standalone launch file, runnable alongside real_robot.launch.py once
parol6-server is up:
    ros2 launch parol6_control_services control_services.launch.py
    ros2 launch parol6_control_services control_services.launch.py robot_port:=5001
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_host = LaunchConfiguration("robot_host")
    robot_port = LaunchConfiguration("robot_port")
    return LaunchDescription([
        DeclareLaunchArgument("robot_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("robot_port", default_value="5001"),
        DeclareLaunchArgument("gripper_io_pin", default_value="0"),
        Node(
            package="parol6_control_services",
            executable="control_services_node",
            output="screen",
            parameters=[{
                "robot_host": robot_host,
                "robot_port": robot_port,
                "gripper_io_pin": LaunchConfiguration("gripper_io_pin"),
            }],
        ),
        Node(
            package="parol6_control_services",
            executable="call_method_node",
            output="screen",
            parameters=[{
                "robot_host": robot_host,
                "robot_port": robot_port,
            }],
        ),
    ])
