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

    # Static transform base_link -> base_footprint
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

    # Nav2 parameters
    nav2_params = str(package_share / "config" / "nav2" / "nav2_params.yaml")
    nav2_bt_navigator = str(package_share / "config" / "nav2" / "nav2_bt_navigator.xml")
    slam_params = str(package_share / "config" / "nav2" / "slam_toolbox_params.yaml")

    # SLAM Toolbox
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params],
    )

    # Nav2 Lifecycle Manager
    # Примечание: map_server не нужен при использовании SLAM, так как SLAM сам публикует карту
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": lifecycle_nodes},
        ],
    )

    # Nav2 Controller Server
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
    )

    # Nav2 Planner Server
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    # Nav2 Behavior Server
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
    )

    # Nav2 BT Navigator
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            nav2_params,
            {"default_nav_to_pose_bt_xml": nav2_bt_navigator},
        ],
    )

    # Nav2 Waypoint Follower
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params],
    )

    # Nav2 Velocity Smoother
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params],
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
            slam_toolbox,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            nav2_lifecycle_manager,
            fake_calibration,
        ]
    )

