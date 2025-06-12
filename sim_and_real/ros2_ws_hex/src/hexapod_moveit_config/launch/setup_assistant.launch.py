"""
Launch file to start the MoveIt Setup Assistant for the hexapod robot.

This launch file initializes the MoveIt Setup Assistant GUI to allow
users to configure and modify the MoveIt configuration for the hexapod robot.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    # Create MoveIt configuration object for the 'hexapod' robot
    moveit_config = MoveItConfigsBuilder("hexapod", package_name="hexapod_moveit_config").to_moveit_configs()
    
    # Launch the MoveIt Setup Assistant GUI for this configuration
    return generate_setup_assistant_launch(moveit_config)
