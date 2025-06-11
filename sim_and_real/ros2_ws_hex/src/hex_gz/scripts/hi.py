#!/usr/bin/env python3


"""
Code responsible for waving the spider with one leg as a greeting
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import matplotlib.animation as animation


matplotlib.use('TkAgg')

spocz_1 = 0
spocz_2 = np.deg2rad(10)
spocz_3 = np.deg2rad(80)

ruch_1_1 = 0
ruch_1_2 = np.deg2rad(-55)
ruch_1_3 = np.deg2rad(-30)

ruch_2_1 = np.deg2rad(-10)
ruch_2_2 = np.deg2rad(-55)
ruch_2_3 = np.deg2rad(-30)

ruch_3_1 = np.deg2rad(10)
ruch_3_2 = np.deg2rad(-55)
ruch_3_3 = np.deg2rad(-30)

# Servo deflections given respectively for joints 1, 2, and 3 in radians
wychyly_serw_podczas_ruchu = np.array([
[[spocz_1, spocz_2, spocz_3], [ruch_1_1, ruch_1_2, ruch_1_3], [ruch_2_1, ruch_2_2, ruch_2_3], [ruch_3_1, ruch_3_2, ruch_3_3], [ruch_2_1, ruch_2_2, ruch_2_3], [ruch_3_1, ruch_3_2, ruch_3_3], [spocz_1, spocz_2, spocz_3]],
[[spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3],[spocz_1, spocz_2, spocz_3]],
[[spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3],[spocz_1, spocz_2, spocz_3]],
[[spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3],[spocz_1, spocz_2, spocz_3]],
[[spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3],[spocz_1, spocz_2, spocz_3]],
[[spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3], [spocz_1, spocz_2, spocz_3],[spocz_1, spocz_2, spocz_3]]
])

class LegSequencePlayer(Node):
    def __init__(self):
        super().__init__('leg_sequence_player')
        self.get_logger().info('Node initialization for movement sequences')
        

        # Publishers for controllers of all legs
        self.trajectory_publishers = {
            1: self.create_publisher(JointTrajectory, '/leg1_controller/joint_trajectory', 10),
            2: self.create_publisher(JointTrajectory, '/leg2_controller/joint_trajectory', 10),
            3: self.create_publisher(JointTrajectory, '/leg3_controller/joint_trajectory', 10),
            4: self.create_publisher(JointTrajectory, '/leg4_controller/joint_trajectory', 10),
            5: self.create_publisher(JointTrajectory, '/leg5_controller/joint_trajectory', 10),
            6: self.create_publisher(JointTrajectory, '/leg6_controller/joint_trajectory', 10)
        }
        
        # Joint lists for legs
        self.joint_names = {
            1: ['joint1_1', 'joint2_1', 'joint3_1'],
            2: ['joint1_2', 'joint2_2', 'joint3_2'],
            3: ['joint1_3', 'joint2_3', 'joint3_3'],
            4: ['joint1_4', 'joint2_4', 'joint3_4'],
            5: ['joint1_5', 'joint2_5', 'joint3_5'],
            6: ['joint1_6', 'joint2_6', 'joint3_6']
        }
        

    def send_trajectory_to_all_legs_at_step(self, step_index, duration_sec=2.0):
        """
        Sends trajectory to controllers of all legs simultaneously
        using values from the wychyly_serw_podczas_ruchu array for the given step
        """
        self.get_logger().info(f'Sending trajectory for step {step_index}')
        
        if step_index >= len(wychyly_serw_podczas_ruchu[0]):
            self.get_logger().error(f'Step index {step_index} is out of range!')
            return False
    
        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        # For each leg prepare and send trajectory
        for leg_num in range(1, 7):
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names[leg_num]
            
            point = JointTrajectoryPoint()
            
            # Get values from array (leg index is leg_num-1)
            joint_values = wychyly_serw_podczas_ruchu[leg_num-1][step_index]
            
            point.positions = [
                float(joint_values[0]), 
                float(joint_values[1]), 
                float(joint_values[2]) 
            ]
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            
            point.time_from_start = duration
            trajectory.points.append(point)
            
            self.trajectory_publishers[leg_num].publish(trajectory)
        
        self.get_logger().info('Sent trajectories for all legs')
        return True
    
    def execute_sequence(self, start_step=0, end_step=None, step_duration=0.5):
        """
        Execute movement sequence for all legs simultaneously
        using values from the wychyly_serw_podczas_ruchu array
        """
        self.get_logger().info('Starting movement sequence for all legs')
        
        if end_step is None:
            end_step = len(wychyly_serw_podczas_ruchu[0])
        
        self.send_trajectory_to_all_legs_at_step(start_step, duration_sec=step_duration)
        self.get_logger().info('Waiting for initial movement execution...')
        time.sleep(step_duration)
        
        for step in range(start_step + 1, end_step):
            self.send_trajectory_to_all_legs_at_step(step, duration_sec=step_duration)
            self.get_logger().info(f'Executed step {step}, waiting {step_duration}s...')
            time.sleep(step_duration)
        
        self.get_logger().info('Sequence completed')


def main(args=None):
    rclpy.init(args=args)
    
    node = LegSequencePlayer()
    
    try:
        print("Initialization... Wait 0.5 seconds.")
        time.sleep(0.5)
        
        # Execute sequence
        print("Starting sequence")
        node.execute_sequence()
        
        time.sleep(0.5)
        
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()