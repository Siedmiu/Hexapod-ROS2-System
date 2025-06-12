"""
Launch file to start the MoveIt warehouse database for the Hexapod robot.

- Uses MoveItConfigsBuilder to load the robot's MoveIt configuration.
- Calls generate_warehouse_db_launch to create and launch the warehouse database node.
- Simplifies management of planning scenes and robot states storage.

This launch file is intended to be included or run alongside other MoveIt-related launch files.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("hexapod", package_name="hexapod_moveit_config").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
