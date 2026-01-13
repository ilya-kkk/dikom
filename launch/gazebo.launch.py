#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    model_default = PathJoinSubstitution(
        [FindPackageShare("amr"), "urdf", "amr.urdf"]
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=model_default,
        description="Absolute path to the robot URDF file.",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        )
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", LaunchConfiguration("model"), "-entity", "amr"],
        output="screen",
    )

    fake_calibration = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "--wait-for-subscription",
            "0",
            "/calibrated",
            "std_msgs/msg/Bool",
            "{data: true}",
        ],
        output="screen",
    )

    return LaunchDescription(
        [model_arg, gazebo, static_tf, spawn_entity, fake_calibration]
    )
