#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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
    package_share = Path(get_package_share_directory("amr"))
    default_model = package_share / "urdf" / "amr.urdf"
    default_rviz = package_share / "config" / "rviz.rviz"
    box_model = package_share / "config" / "boxes" / "box.sdf"
    ground_plane_model = package_share / "config" / "ground_plane.sdf"
    
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=str(default_model),
        description="Absolute path to the robot URDF file.",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start Gazebo client GUI (gzclient). If false, runs headless (gzserver only).",
    )
    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=str(default_rviz),
        description="RViz config file.",
    )
    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="true",
        description="Start teleop_twist_keyboard for /cmd_vel control.",
    )

    # Gazebo
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

    # Static transform base_link -> base_footprint
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
        output="screen",
    )

    # Spawn ground plane
    spawn_ground = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            str(ground_plane_model),
            "-entity",
            "ground_plane",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            LaunchConfiguration("model"),
            "-entity",
            "amr",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.5",
            "-Y",
            "0.7",
        ],
        output="screen",
    )

    # Static obstacles: 4 boxes (0.1 x 0.1 x 0.3), square 1m side shifted +2m in X
    box1 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            str(box_model),
            "-entity",
            "box1",
            "-x",
            "2.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )
    box2 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            str(box_model),
            "-entity",
            "box2",
            "-x",
            "3.5",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )
    box3 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            str(box_model),
            "-entity",
            "box3",
            "-x",
            "2.0",
            "-y",
            "1.5",
            "-z",
            "0.0",
        ],
        output="screen",
    )
    box4 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            str(box_model),
            "-entity",
            "box4",
            "-x",
            "3.5",
            "-y",
            "1.5",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    # Another 4 boxes: square 0.8m side shifted +2m in Y
    box5 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            str(box_model),
            "-entity",
            "box5",
            "-x",
            "0.0",
            "-y",
            "2.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )
    box6 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            str(box_model),
            "-entity",
            "box6",
            "-x",
            "0.8",
            "-y",
            "2.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )
    box7 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            str(box_model),
            "-entity",
            "box7",
            "-x",
            "0.0",
            "-y",
            "2.8",
            "-z",
            "0.0",
        ],
        output="screen",
    )
    box8 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            str(box_model),
            "-entity",
            "box8",
            "-x",
            "0.8",
            "-y",
            "2.8",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    # Robot State Publisher
    robot_state_publisher = OpaqueFunction(function=_robot_state_publisher_setup)

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        output="screen",
    )

    # Teleop Twist Keyboard (publishes to /cmd_vel)
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        emulate_tty=True,
        remappings=[("cmd_vel", "/cmd_vel")],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )

    # Rack Finder Service node
    rack_finder_service = Node(
        package="amr",
        executable="test.py",
        name="rack_finder_service",
        output="screen",
        arguments=["--ros-args", "--log-level", "rack_finder_service:=debug"],
        parameters=[
            {"scan_topic": "scan"},
            {"diff_threshold": 0.15},
            {"max_width_rays": 30},
            {"max_legs": 6},
            {"range_max": 0.0},
        ],
    )

    delayed_spawns = TimerAction(
        period=5.0,
        actions=[spawn_ground, spawn_entity, box1, box2, box3, box4, box5, box6, box7, box8],
    )

    return LaunchDescription(
        [
            model_arg,
            gui_arg,
            rviz_arg,
            teleop_arg,
            gzserver,
            gzclient,
            static_tf,
            delayed_spawns,
            robot_state_publisher,
            rviz,
            teleop,
            rack_finder_service,
        ]
    )

