from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generates the launch description for the VIPS driver node.
    This launch file configures the serial port for communication.
    """

    # Declare a launch argument for the serial port device name.
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="The serial port connected to the VIPS Rover (e.g., /dev/ttyUSB0).",
    )

    # Define the node to be launched.
    vips_driver_node = Node(
        package="vips_driver",  # The name of your ROS 2 package
        executable="vips_node",  # The name of the executable created by CMakeLists.txt
        name="vips_node",  # The name of the node as it will appear in the ROS 2 graph
        output="screen",  # Show node output directly in the terminal
        emulate_tty=True,  # Helps with seeing colorized log messages
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
            }
        ],
    )

    # Return the complete launch description
    return LaunchDescription([serial_port_arg, vips_driver_node])
