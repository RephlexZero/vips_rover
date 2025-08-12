"""
DEPRECATED: Use rover_description/launch/simulation.launch.py instead

This file is kept for backward compatibility but should be updated to use
the new separated rover_description package.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Redirect to the new simulation launch file in rover_description package
    """

    pkg_rover_description = get_package_share_directory("rover_description")

    # Include the new simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_description, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world", default="empty.sdf"),
            "urdf_model": LaunchConfiguration(
                "urdf_model",
                default=os.path.join(pkg_rover_description, "urdf/rover.urdf.xacro"),
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="empty.sdf"),
            DeclareLaunchArgument(
                "urdf_model",
                default_value=os.path.join(
                    pkg_rover_description, "urdf/rover.urdf.xacro"
                ),
            ),
            simulation_launch,
        ]
    )
