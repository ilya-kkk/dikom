#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    package_share = Path(get_package_share_directory("amr"))
    default_model = package_share / "urdf" / "amr.urdf"
    
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=str(default_model),
        description="Absolute path to the robot URDF file.",
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        )
    )

    # Static transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
        output="screen",
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", LaunchConfiguration("model"), "-entity", "amr"],
        output="screen",
    )

    # Robot State Publisher
    robot_state_publisher = OpaqueFunction(function=_robot_state_publisher_setup)

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    # Rack Finder Service node
    rack_finder_service = Node(
        package="amr",
        executable="test.py",
        name="rack_finder_service",
        output="screen",
        parameters=[
            {"scan_topic": "scan"},
            {"diff_threshold": 0.15},
            {"max_width_rays": 30},
            {"max_legs": 6},
            {"range_max": 0.0},
        ],
    )

    # Fake calibration (if needed)
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
            gazebo,
            static_tf,
            spawn_entity,
            robot_state_publisher,
            rviz,
            rack_finder_service,
            fake_calibration,
        ]
    )

