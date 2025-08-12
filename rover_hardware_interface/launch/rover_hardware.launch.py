"""
DEPRECATED: Use rover_description/launch/hardware.launch.py instead

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
    Redirect to the new hardware launch file in rover_description package
    """

    pkg_rover_description = get_package_share_directory("rover_description")

    # Include the new hardware launch file
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_description, "launch", "hardware.launch.py")
        ),
        launch_arguments={
            "urdf_model": LaunchConfiguration(
                "urdf_model",
                default=os.path.join(pkg_rover_description, "urdf/rover.urdf.xacro"),
            ),
            "vips_serial_port": LaunchConfiguration(
                "vips_serial_port", default="/dev/ttyUSB0"
            ),
            "can_interface": LaunchConfiguration("can_interface", default="can0"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_model",
                default_value=os.path.join(
                    pkg_rover_description, "urdf/rover.urdf.xacro"
                ),
            ),
            DeclareLaunchArgument("vips_serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            hardware_launch,
        ]
    )
