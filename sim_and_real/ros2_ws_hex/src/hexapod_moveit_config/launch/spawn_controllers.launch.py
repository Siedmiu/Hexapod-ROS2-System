"""
Launch file to spawn the robot controllers for the hexapod robot MoveIt configuration.

This file sets up and launches the necessary controller nodes to manage
hardware interfaces or simulated joints for the hexapod robot.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    # Build MoveIt configuration for the 'hexapod' robot from the specified package
    moveit_config = MoveItConfigsBuilder("hexapod", package_name="hexapod_moveit_config").to_moveit_configs()
    
    # Generate and return the launch description to spawn the robot controllers
    return generate_spawn_controllers_launch(moveit_config)
