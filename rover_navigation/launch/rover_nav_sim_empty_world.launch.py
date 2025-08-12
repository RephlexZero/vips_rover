#!/usr/bin/env python3

"""
Complete rover navigation launch for an empty world (no SLAM, no lidar)
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
    rover_nav_dir = get_package_share_directory("rover_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )

    # Include rover simulation launch (robot + controllers), lidar disabled by default in URDF arg
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

    # Include Empty-World Navigation (no SLAM)
    nav2_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_nav_dir, "launch", "navigation_empty_world.launch.py")
        ),
        launch_arguments=[("use_sim_time", use_sim_time)],
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

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(rover_simulation_launch)
    ld.add_action(nav2_empty_world)
    ld.add_action(rviz_node)
    return ld
