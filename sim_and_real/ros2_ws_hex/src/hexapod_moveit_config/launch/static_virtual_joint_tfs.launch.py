"""
Launch file to start static transforms for virtual joints in the hexapod MoveIt configuration.

This file generates the necessary static TFs (transform frames) for virtual joints
defined in the 'hexapod_moveit_config' package, which MoveIt uses for kinematic setup.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    # Build MoveIt configuration for the 'hexapod' robot from the specified package
    moveit_config = MoveItConfigsBuilder("hexapod", package_name="hexapod_moveit_config").to_moveit_configs()
    
    # Generate and return the launch description to start static virtual joint TF publishers
    return generate_static_virtual_joint_tfs_launch(moveit_config)
