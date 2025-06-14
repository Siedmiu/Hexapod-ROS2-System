#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Pobieranie ścieżek do pakietów
    hexapod_description_path = get_package_share_directory('hexapod_description')
    hex_gz_path = get_package_share_directory('hex_gz')
    
    # Argumenty uruchomieniowe
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty')
    
    # Deklaracja argumentów
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty_ode',
        description='Gazebo World file name'
    )
    
    # Ustawienie ścieżki zasobów dla Gazebo
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(hex_gz_path, 'worlds'),
            ':' + str(Path(hexapod_description_path).parent.resolve()),
        ],
    )

    # Uruchomienie Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments=[
            ('gz_args', [world, '.sdf', ' -v 4', ' -r']),
        ],
    )

    # Przetwarzanie pliku XACRO
    xacro_file = os.path.join(
        hexapod_description_path,
        'urdf',
        'hexapod.urdf.xacro',
    )

    # Parsowanie pliku XACRO z parametrem use_sim=true
    doc = xacro.process_file(xacro_file, mappings={'use_gazebo': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    
    # Parametry dla robot_state_publisher
    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

    # Uruchomienie robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # Bridge z dodanymi sensorami kontaktu
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/empty/model/hexapod/link/link4_1/sensor/sensor_contact_1/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/empty/model/hexapod/link/link4_2/sensor/sensor_contact_2/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/empty/model/hexapod/link/link4_3/sensor/sensor_contact_3/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/empty/model/hexapod/link/link4_4/sensor/sensor_contact_4/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/empty/model/hexapod/link/link4_5/sensor/sensor_contact_5/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/empty/model/hexapod/link/link4_6/sensor/sensor_contact_6/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        output='screen',
    )

    # Contact Detection Node
    contact_detection_node = Node(
        package='hexapod_description',
        executable='contact_sensor.py',
        name='contact_sensor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Spawning modelu w Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name', 'hexapod',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',  
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-allow_renaming', 'true',
        ],
    )

    # Dodanie krótkiej pauzy przed ładowaniem kontrolerów
    initial_pause = ExecuteProcess(
        cmd=['sleep', '5.0'],
        output='screen',
    )

    # Ładowanie joint_state_broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', 
            'joint_state_broadcaster',
            '--set-state', 'active',
        ],
        output='screen',
    )

    # Ładowanie i aktywacja kontrolerów nóg
    load_leg1_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', 
            'leg1_controller',
            '--set-state', 'active',
        ],
        output='screen',
    )
    
    load_leg2_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', 
            'leg2_controller',
            '--set-state', 'active',
        ],
        output='screen',
    )
    
    load_leg3_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', 
            'leg3_controller',
            '--set-state', 'active',
        ],
        output='screen',
    )
    
    load_leg4_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', 
            'leg4_controller',
            '--set-state', 'active',
        ],
        output='screen',
    )
    
    load_leg5_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', 
            'leg5_controller',
            '--set-state', 'active',
        ],
        output='screen',
    )
    
    load_leg6_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', 
            'leg6_controller',
            '--set-state', 'active',
        ],
        output='screen',
    )

    # Tworzenie sekwencji uruchomienia
    launch_sequence = [
        gazebo_resource_path,
        declare_use_sim_time,
        declare_world,
        gazebo,
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        contact_detection_node,  # Dodanie contact detection node
    ]
    
    # Rejestracja zdarzenia do ładowania joint_state_broadcaster z opóźnieniem
    launch_sequence.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[initial_pause, load_joint_state_broadcaster],
            )
        )
    )
    
    # Rejestracja zdarzenia do ładowania kontrolerów nóg po załadowaniu joint_state_broadcaster
    launch_sequence.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[
                    load_leg1_controller,
                    load_leg2_controller,
                    load_leg3_controller,
                    load_leg4_controller,
                    load_leg5_controller,
                    load_leg6_controller
                ],
            )
        )
    )
    
    return LaunchDescription(launch_sequence)