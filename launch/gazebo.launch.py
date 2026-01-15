#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _robot_state_publisher_setup(context, *args, **kwargs):
    model_path = Path(LaunchConfiguration("model").perform(context))
    robot_description = model_path.read_text()
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    model_default = PathJoinSubstitution(
        [FindPackageShare("amr"), "urdf", "amr.urdf"]
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=model_default,
        description="Absolute path to the robot URDF file.",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start Gazebo client GUI (gzclient). If false, runs headless (gzserver only).",
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "worlds", "empty.world"]
    )

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            world_path,
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen",
    )
    gzclient = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("gui")),
        cmd=["gzclient", "--verbose"],
        output="screen",
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", LaunchConfiguration("model"), "-entity", "amr", "-z", "0.5"],
        output="screen",
    )
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])

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
        [
            model_arg,
            gui_arg,
            OpaqueFunction(function=_robot_state_publisher_setup),
            static_tf,
            gzserver,
            gzclient,
            delayed_spawn,
            fake_calibration,
        ]
    )
