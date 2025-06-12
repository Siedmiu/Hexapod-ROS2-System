"""
Launch file to start the MoveIt Runtime Setup Panel (RSP) for the hexapod robot.

This launch file opens the MoveIt RSP GUI, which allows users to 
interactively manage controllers, planning groups, and other runtime settings
for the hexapod MoveIt configuration.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    # Create MoveIt configuration object for the 'hexapod' robot
    moveit_config = MoveItConfigsBuilder("hexapod", package_name="hexapod_moveit_config").to_moveit_configs()
    
    # Launch the MoveIt Runtime Setup Panel for this configuration
    return generate_rsp_launch(moveit_config)
