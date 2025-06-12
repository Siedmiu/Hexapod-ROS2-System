"""
Launch file to start RViz with MoveIt configuration for the hexapod robot.

This launch file initializes the MoveIt planning environment and opens RViz,
pre-configured with the hexapod robot’s MoveIt settings for visualization
and interactive motion planning.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    # Build MoveIt configuration for the 'hexapod' robot
    moveit_config = MoveItConfigsBuilder("hexapod", package_name="hexapod_moveit_config").to_moveit_configs()
    
    # Launch RViz with the MoveIt configuration
    return generate_moveit_rviz_launch(moveit_config)
