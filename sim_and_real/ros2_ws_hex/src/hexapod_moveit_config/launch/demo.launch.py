"""
Launch file to start a MoveIt demo for the hexapod robot.

This launch file runs a pre-configured demo that usually
includes RViz with MoveIt plugins, robot state visualization,
and example motion planning scenarios.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Build MoveIt configuration for the 'hexapod' robot
    moveit_config = MoveItConfigsBuilder("hexapod", package_name="hexapod_moveit_config").to_moveit_configs()
    
    # Launch the demo with the configured parameters
    return generate_demo_launch(moveit_config)
