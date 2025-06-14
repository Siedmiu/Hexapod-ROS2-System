#!/usr/bin/env python3
"""
Hexapod robot display launch file.

This launch file provides visualization setup for the hexapod robot in RViz.
It loads the robot model, publishes joint states, and launches RViz with
a predefined configuration for robot monitoring and visualization.

Usage:
    # Basic visualization with GUI sliders
    ros2 launch hexapod_description hexapod_display.launch.py
    
    # Without GUI joint control
    ros2 launch hexapod_description hexapod_display.launch.py gui:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for hexapod robot visualization.
    
    Sets up robot state publisher, joint state publisher, and RViz
    for complete hexapod robot visualization and monitoring.
    
    Returns:
        LaunchDescription: Complete visualization setup
    """
    
    # Launch argument declarations for display configuration
    declared_arguments = [
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start joint state publisher GUI for interactive control',
        ),
    ]

    # Launch configuration retrieval
    gui = LaunchConfiguration('gui')

    # Robot description generation from XACRO with display-specific parameters
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
        '""',                        # No prefix for single robot
        ' ',
        'use_gazebo:=',
        'false',                     # Display mode, not simulation
        ' ',
        'use_fake_hardware:=',
        'true',                      # Always fake hardware for display
        ' ',
        'fake_sensor_commands:=',
        'true',                      # Always fake sensors for display
        ' ',
        'port_name:=',
        '/dev/ttyUSB0',              # Default (unused in display)
        ' ',
        'baud_rate:=',
        '115200',                    # Default (unused in display)
    ])

    # Robot description parameter for all nodes
    robot_description = {'robot_description': urdf_file}

    # RViz configuration file path
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('hex_gz'),  # Hexapod bringup package (launch files, scripts, configs)
        'rviz',
        'hexapod.rviz',              # RViz visualization configuration
    ])

    # Robot state publisher - publishes joint states to TF tree
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': False}],
        output='screen',
    )

    # Joint state publisher with GUI - interactive joint control with sliders
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'rate': 30,                      # Publishing frequency for smooth updates
            'source_list': ['joint_states']  # Listen to joint_states topic
        }],
        condition=IfCondition(gui),
    )

    # Joint state publisher without GUI - basic joint state publishing
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'rate': 30,                      # Publishing frequency
            'source_list': ['joint_states']  # Listen to joint_states topic
        }],
        condition=UnlessCondition(gui),
    )

    # RViz visualization node - displays robot model and joint states
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # Launch description assembly
    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,      # Robot state publishing
            joint_state_publisher_gui_node,  # Interactive joint control (conditional)
            joint_state_publisher_node,      # Basic joint publishing (conditional)
            rviz_node,                       # Visualization interface
        ]
    )