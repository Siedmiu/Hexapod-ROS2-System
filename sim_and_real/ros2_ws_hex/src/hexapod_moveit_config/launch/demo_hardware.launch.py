"""
Launch file to start a MoveIt demo for the 'hexapod_hardware' robot.

This launch runs a demo setup, including RViz with MoveIt
interfaces and example motion planning for the physical robot hardware.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Build MoveIt configuration for 'hexapod_hardware' robot
    moveit_config = MoveItConfigsBuilder("hexapod_hardware", package_name="hexapod_moveit_config").to_moveit_configs()
    
    # Launch the demo with hardware-specific configuration
    return generate_demo_launch(moveit_config)
