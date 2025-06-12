"""
Launch file to start the MoveIt Move Group node for the hexapod robot.

This launch file initializes the Move Group, which is the core
motion planning component in MoveIt. It handles planning,
trajectory execution, and interaction with the robot hardware or simulation.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    # Build MoveIt configuration for the 'hexapod' robot
    moveit_config = MoveItConfigsBuilder("hexapod", package_name="hexapod_moveit_config").to_moveit_configs()
    
    # Launch the Move Group node with the configured parameters
    return generate_move_group_launch(moveit_config)
