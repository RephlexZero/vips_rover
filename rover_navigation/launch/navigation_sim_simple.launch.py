#!/usr/bin/env python3

"""
Simplified launch file for rover navigation in simulation mode with SLAM
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    # Include Nav2 bringup with localization
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments=[
            ("use_sim_time", use_sim_time),
            ("slam", "True"),
            (
                "params_file",
                os.path.join(rover_nav_dir, "config", "nav2_params_sim.yaml"),
            ),
        ],
    )

    # Cmd_vel to Ackermann converter
    cmd_vel_converter = Node(
        package="rover_navigation",
        executable="cmd_vel_to_ackermann.py",
        name="cmd_vel_to_ackermann",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/cmd_vel", "/cmd_vel"),
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

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes and launch files
    ld.add_action(rover_simulation_launch)
    ld.add_action(nav2_launch)
    ld.add_action(cmd_vel_converter)
    ld.add_action(rviz_node)

    return ld
