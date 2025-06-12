#!/usr/bin/env python3

"""
Contact Detection Node for Hexapod Simulation in Gazebo

This ROS 2 node listens to contact sensor data from Gazebo simulation for each of the hexapod's legs.
Each leg has a contact sensor located at the bottom, used to detect when the leg touches the ground.

The node subscribes to Gazebo topics providing contact information and republishes a simplified Boolean
status (`True` if contact detected) to a separate topic for each leg under `/hexapod/legX/contact_status`.

The purpose is to enable other components (e.g., locomotion controller, gait planner) to react to contact
events in simulation and adjust behavior accordingly, such as triggering stance/swing phase transitions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ros_gz_interfaces.msg import Contacts


class ContactDetectionNode(Node):
    def __init__(self):
        super().__init__('contact_detection_node')
        
        # Dictionaries to manage subscriptions, publishers, and contact flags for each leg
        self.contact_subscribers = {}
        self.contact_publishers = {}
        self.received_flags = {}
        self.message_counts = {}
        
        # Create subscriptions and publishers for 6 legs of the hexapod
        for leg_num in range(1, 7):
            # Gazebo topic for contact sensor data
            gazebo_topic = f'/world/empty/model/hexapod/link/link4_{leg_num}/sensor/sensor_contact_{leg_num}/contact'
            
            # ROS topic to publish contact status
            ros_topic = f'/hexapod/leg{leg_num}/contact_status'
            
            # Subscriber listening to contact messages from Gazebo
            self.contact_subscribers[leg_num] = self.create_subscription(
                Contacts,
                gazebo_topic,
                lambda msg, ln=leg_num: self.contact_callback(msg, ln),
                10
            )
            
            # Publisher sending out contact status as Bool
            self.contact_publishers[leg_num] = self.create_publisher(
                Bool,
                ros_topic,
                10
            )
            
            # Initialize contact flag and message counter for each leg
            self.received_flags[leg_num] = False
            self.message_counts[leg_num] = 0
        
        # Timer triggering periodic publishing (every 100 ms)
        self.publish_timer = self.create_timer(0.1, self.publish_all_status)
        
        # Counter to track number of publish events (optional/debugging)
        self.publish_count = 0
        
    def contact_callback(self, msg, leg_number):
        #Callback triggered when a contact message is received from Gazebo for a specific leg
        self.message_counts[leg_number] += 1
        self.received_flags[leg_number] = True
        
    def publish_all_status(self):
        #Periodically publishes the contact status for all legs
        self.publish_count += 1
        
        # Publikuj status dla każdej nogi
        for leg_num in range(1, 7):
            # Create a Bool message indicating contact status
            msg = Bool()
            msg.data = self.received_flags[leg_num]
            
            # Publish contact status
            self.contact_publishers[leg_num].publish(msg)
            
            # Reset flag after publishing (clears detection state)
            self.received_flags[leg_num] = False


def main(args=None):
    rclpy.init(args=args)
    node = ContactDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()