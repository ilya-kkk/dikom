#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_state_publisher_setup(context, *args, **kwargs):
    model_path = Path(LaunchConfiguration("model").perform(context))
    robot_description = model_path.read_text()
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("amr"))
    default_model = package_share / "urdf" / "amr.urdf"
    default_rviz = package_share / "config" / "rviz.rviz"

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=str(default_model),
        description="Absolute path to the robot URDF file.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=str(default_rviz),
        description="RViz config file.",
    )

    return LaunchDescription(
        [
            model_arg,
            rviz_arg,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            OpaqueFunction(function=_robot_state_publisher_setup),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
            ),
        ]
    )
