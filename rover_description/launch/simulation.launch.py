"""
Pure simulation launch file - NO HARDWARE COMPONENTS

This launch file is specifically designed for simulation testing and excludes:
- VIPS driver
- Hardware CAN interfaces
- Hardware-specific configurations

Use this for:
- Algorithm development
- Controller tuning
- Path planning testing
- General rover simulation
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    
    # Try to find ros_gz_sim, fall back to gazebo_ros if using older distribution
    try:
        pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
        use_new_gazebo = True
    except:
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
        use_new_gazebo = False

    # --- Launch Arguments ---
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value='empty.sdf',
        description='Gazebo world file to load'
    )
    
    urdf_model_arg = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=os.path.join(pkg_share, 'urdf/rover.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # --- Robot Description with SIMULATION parameters ---
    robot_description = ParameterValue(
        Command([
            'xacro ', LaunchConfiguration('urdf_model'),
            ' hardware_plugin:=gz_ros2_control/GazeboSimSystem',
            ' controllers_file:=rover_controllers_sim.yaml',
            ' use_sim_time:=true'
        ]), 
        value_type=str
    )

    # --- Gazebo Simulation Launch ---
    if use_new_gazebo:
        # Modern Gazebo (Ignition/Garden)
        gz_sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments=[
                ('gz_args', ['-r -v4 ', LaunchConfiguration('world')]),
                ('use_sim_time', 'true')
            ]
        )
        
        # Clock bridge for new Gazebo
        clock_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
        
        # Spawn robot in new Gazebo
        spawn_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'ackermann_rover'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
        
        extra_nodes = [clock_bridge, spawn_entity_node]
        
    else:
        # Classic Gazebo
        gz_sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'use_sim_time': 'true'
            }.items()
        )
        
        # Spawn robot in classic Gazebo
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'ackermann_rover'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
        
        extra_nodes = [spawn_entity_node]

    # --- Core Nodes ---
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description
        }]
    )

    # --- Controller Spawners (Delayed) ---
    # Even with ros_gz_sim, explicitly spawning ensures controllers are active
    
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,  # Wait for Gazebo and controller manager
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    ackermann_controller_spawner = TimerAction(
        period=4.0,  # Wait for joint state broadcaster
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Build launch description
    launch_nodes = [
        # Arguments
    world_arg,
        urdf_model_arg,
    use_sim_time_arg,
        
        # Core simulation
        gz_sim_launch,
        robot_state_publisher_node,
        
        # Ensure controllers are spawned and activated
        joint_state_broadcaster_spawner,
        ackermann_controller_spawner,
    ]
    
    # Add extra nodes based on Gazebo version
    launch_nodes.extend(extra_nodes)

    return LaunchDescription(launch_nodes)
