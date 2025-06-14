#!/usr/bin/env python3
"""
ROS2 Launch file for visualizing hexapod collision geometry in RViz.

This launch file loads a hexapod robot model from XACRO format and configures RViz
to display the collision or visual geometry for debugging purposes.
It provides a complete visualization setup including robot state publishers
and joint state control for interactive model manipulation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Generate launch description for hexapod collision visualization.
    
    Returns:
        LaunchDescription: Complete launch configuration with all necessary nodes
    """
    
    # Package configuration - define hexapod description and simulation packages
    package_description_name = 'hexapod_description'  # Main robot description package
    package_gz_name = 'hex_gz'  # Hexapod bringup package (launch files, scripts, configs)

    # Get package directories and set up file paths
    pkg_share_description = get_package_share_directory(package_description_name)
    pkg_share_gz = get_package_share_directory(package_gz_name)
    default_model_path = os.path.join(pkg_share_description, 'urdf', 'hexapod.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share_gz, 'rviz', 'collision_view.rviz')
    
    # Launch arguments
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Path to the XACRO model file for the hexapod robot'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config', 
        default_value=default_rviz_config_path,
        description='Path to RViz configuration file for collision visualization'
    )
    
    gui_arg = DeclareLaunchArgument(
        name='gui', 
        default_value='true',
        description='Launch joint state publisher GUI for interactive joint control'
    )
    
    # Get launch configuration values
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    gui = LaunchConfiguration('gui')
    
    # Process XACRO file to generate URDF robot description
    robot_description = ParameterValue(
        Command(['xacro ', model]),  # Execute xacro command to process the model
        value_type=str
    )
    
    # Node definitions
    
    # Joint State Publisher - publishes joint states and optionally provides GUI
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': gui}],
        output='screen'
    )
    
    # Robot State Publisher - transforms joint states into TF transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # RViz - visualization with collision-specific configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Return complete launch description
    return LaunchDescription([
        model_arg,
        rviz_arg,
        gui_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])