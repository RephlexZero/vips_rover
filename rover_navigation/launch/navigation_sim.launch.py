#!/usr/bin/env python3

"""
Simplified launch file for rover navigation in simulation mode with SLAM
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rover_nav_dir = get_package_share_directory("rover_navigation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # Parameters
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )

    # Include rover simulation launch (robot + controllers)
    rover_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("rover_description"),
                        "launch",
                        "simulation.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("use_sim_time", use_sim_time),
        ],
    )

    # Start a static map->odom TF BEFORE Nav2 to avoid early TF timeouts
    static_map_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # Include Nav2 bringup with localization disabled (no lidar) and our tuned params
    nav2_launch = GroupAction(
        [
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[
                    os.path.join(rover_nav_dir, "config", "nav2_params_sim.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[
                    os.path.join(rover_nav_dir, "config", "nav2_params_sim.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[
                    os.path.join(rover_nav_dir, "config", "nav2_params_sim.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[
                    os.path.join(rover_nav_dir, "config", "nav2_params_sim.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    os.path.join(rover_nav_dir, "config", "nav2_params_sim.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[
                    os.path.join(rover_nav_dir, "config", "nav2_params_sim.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[
                    os.path.join(rover_nav_dir, "config", "nav2_params_sim.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
                # Correct remapping: in=/cmd_vel, out=/cmd_vel_smoothed
                remappings=[
                    ("/cmd_vel", "/cmd_vel"),  # subscribe default
                    (
                        "/cmd_vel_smoothed",
                        "/cmd_vel_smoothed",
                    ),  # publish default topic name
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {
                        "node_names": [
                            "controller_server",
                            "smoother_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                            "waypoint_follower",
                            "velocity_smoother",
                        ]
                    },
                ],
            ),
        ]
    )

    # Cmd_vel to Ackermann converter subscribes to smoothed twist
    cmd_vel_converter = Node(
        package="rover_navigation",
        executable="cmd_vel_to_ackermann.py",
        name="cmd_vel_to_ackermann",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/cmd_vel", "/cmd_vel_smoothed"),
            ("/ackermann_cmd", "/ackermann_steering_controller/reference_unstamped"),
        ],
    )

    # RViz2 with navigation config
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rover_navigation"), "config", "nav2_default_view.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Ensure TF exists before Nav2
    ld.add_action(rover_simulation_launch)
    ld.add_action(static_map_odom_tf)
    ld.add_action(nav2_launch)
    ld.add_action(cmd_vel_converter)
    ld.add_action(rviz_node)

    return ld
