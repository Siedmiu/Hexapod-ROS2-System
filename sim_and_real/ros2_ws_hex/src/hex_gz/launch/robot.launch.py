#!/usr/bin/env python3
"""
Hexapod robot hardware interface launch file.

This launch file establishes communication with the physical hexapod robot hardware
and initializes the ros2_control system for real robot operation. It sets up the
hardware interface, loads joint controllers, and optionally starts RViz for monitoring.
This is the primary launch file that should be started before running motion algorithms.

Usage:
    # Basic hardware startup
    ros2 launch hexapod_description robot.launch.py
    
    # With RViz monitoring
    ros2 launch hexapod_description robot.launch.py start_rviz:=true
    
    # Custom serial port
    ros2 launch hexapod_description robot.launch.py port_name:=/dev/ttyACM0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for hexapod hardware interface.
    
    Sets up the complete hardware communication stack including ros2_control node,
    robot state publisher, and individual leg controllers for the physical robot.
    
    Returns:
        LaunchDescription: Complete hardware interface setup
    """
    
    # Launch argument declarations for hardware configuration
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', 
            default_value='false', 
            description='Whether to execute rviz2 for robot monitoring'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names for multi-robot setups',
        ),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='Start robot in Gazebo simulation (false for real hardware)',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware mirroring command for testing',
        ),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Enable fake sensor commands for debugging',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port name for ESP32 communication',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication with microcontroller',
        ),
    ]

    # Launch configuration retrieval
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    baud_rate = LaunchConfiguration('baud_rate')

    # Robot description generation from XACRO with hardware-specific parameters
    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('hexapod_description'),
            'urdf',
            'hexapod.urdf.xacro',
        ]),
        ' ',
        'prefix:=',
        prefix,                    # Robot naming prefix
        ' ',
        'use_gazebo:=',
        use_gazebo,               # Hardware or simulation mode
        ' ',
        'use_fake_hardware:=',
        use_fake_hardware,        # Real or fake hardware interface
        ' ',
        'fake_sensor_commands:=',
        fake_sensor_commands,     # Sensor simulation mode
        ' ',
        'port_name:=',
        port_name,               # Serial communication port
        ' ',
        'baud_rate:=',
        baud_rate,               # Serial communication speed
        ' ',
    ])

    # Robot description parameter for all nodes
    robot_description = {'robot_description': urdf_file}

    # Configuration file paths
    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('hexapod_description'),
        'control',
        'controller_manager.yaml',    # ros2_control configuration
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('hex_gz'),
        'rviz',
        'hexapod.rviz',              # RViz visualization configuration
    ])

    # Hardware interface node - communicates with physical robot
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_gazebo),  # Only for real hardware, not simulation
    )

    # Robot state publisher - publishes joint states to TF tree
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_gazebo}],
        output='screen',
    )

    # RViz visualization node - optional monitoring interface
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    # Controller spawner - loads and activates individual leg controllers
    # Each leg has dedicated controller for independent joint control
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'leg1_controller',           # Front right leg controller
            'leg2_controller',           # Middle right leg controller
            'leg3_controller',           # Rear right leg controller
            'leg4_controller',           # Rear left leg controller
            'leg5_controller',           # Middle left leg controller
            'leg6_controller',           # Front left leg controller
            'joint_state_broadcaster',   # Joint state publishing controller
        ],
        parameters=[robot_description],
    )

    # Event-driven startup sequence - ensures proper initialization order
    delay_rviz_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner, 
            on_exit=[rviz_node]                     # Start RViz only after controllers are loaded
        )
    )

    # Launch description assembly with sequential startup
    return LaunchDescription(
        declared_arguments
        + [
            control_node,                           # Hardware communication interface
            robot_controller_spawner,               # Controller initialization
            robot_state_publisher_node,             # State broadcasting
            delay_rviz_after_controller_spawner,    # Optional visualization
        ]
    )