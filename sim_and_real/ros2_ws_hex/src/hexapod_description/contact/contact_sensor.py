#!/usr/bin/env python3

"""
ROS2 node for processing hexapod contact sensor signals from Gazebo simulation.

This node acts as a bridge between Gazebo contact sensor messages and standard
ROS2 topics, enabling easy integration with the robot control system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ros_gz_interfaces.msg import Contacts


class ContactDetectionNode(Node):
    """
    ROS2 node responsible for detecting hexapod leg contact with the ground.
    
    Subscribes to Gazebo contact sensor messages for each of the 6 robot legs
    and publishes simplified Bool messages on dedicated ROS2 topics.
    
    Input topics (Gazebo):
        /world/empty/model/hexapod/link/link4_{1-6}/sensor/sensor_contact_{1-6}/contact
        
    Output topics (ROS2):
        /hexapod/leg{1-6}/contact_status
        
    The node operates at 10Hz publishing frequency.
    """

    def __init__(self):
        """
        Initialize the contact detection node.
        
        Sets up subscribers for all 6 leg contact sensors, publishers for
        contact status topics, and a timer for periodic status publishing.
        """
        super().__init__('contact_detection_node')
        
        # Dictionaries to store subscriptions, publishers and flags for each leg
        self.contact_subscribers = {}  # Gazebo contact sensor subscriptions
        self.contact_publishers = {}   # ROS2 contact status publishers
        self.received_flags = {}       # Contact detection flags (True if contact detected)
        self.message_counts = {}       # Debug counters for received messages
        
        # Configuration for each leg (1-6)
        for leg_num in range(1, 7):
            # Gazebo contact sensor topic for leg N
            gazebo_topic = f'/world/empty/model/hexapod/link/link4_{leg_num}/sensor/sensor_contact_{leg_num}/contact'
            
            # ROS2 topic for publishing contact status
            ros_topic = f'/hexapod/leg{leg_num}/contact_status'
            
            # Create subscription to Gazebo contact sensor
            # Using lambda with default parameter to capture leg_num correctly
            self.contact_subscribers[leg_num] = self.create_subscription(
                Contacts,
                gazebo_topic,
                lambda msg, ln=leg_num: self.contact_callback(msg, ln),
                10 
            )
            
            # Create publisher for contact status
            self.contact_publishers[leg_num] = self.create_publisher(
                Bool,
                ros_topic,
                10 
            )
            
            # Initialize flags and counters for this leg
            self.received_flags[leg_num] = False  
            self.message_counts[leg_num] = 0    
        
        # Timer for regular status publishing (every 100ms = 10Hz)
        self.publish_timer = self.create_timer(0.1, self.publish_all_status)
        
        # Debug counter for total publications
        self.publish_count = 0
        
        self.get_logger().info('Contact detection node initialized for 6 legs')

    def contact_callback(self, msg, leg_number):
        """
        Callback function triggered when contact message is received from Gazebo.
        
        Args:
            msg (Contacts): Gazebo contact message containing collision information
            leg_number (int): Leg number (1-6) that detected contact
            
        Note:
            Sets the contact flag to True for the specific leg. The flag will be
            changed back to False after publishing the status in the periodic timer.
        """
        self.message_counts[leg_number] += 1
        self.received_flags[leg_number] = True

    def publish_all_status(self):
        """
        Publish contact status for all legs and reset contact flags.
        
        Called periodically by the timer (10Hz). Publishes Bool messages
        indicating whether each leg has detected contact since the last
        publication cycle, then resets all flags to False.
        
        This approach ensures that contact detection is event-based but
        published at a consistent rate for the control system.
        """
        self.publish_count += 1
        
        # Publish status for each leg
        for leg_num in range(1, 7):
            # Create Bool message with current contact status
            msg = Bool()
            msg.data = self.received_flags[leg_num]
            
            # Publish the contact status
            self.contact_publishers[leg_num].publish(msg)
            
            # Reset flag after publication (contact detection is edge-triggered)
            self.received_flags[leg_num] = False
        
        # Optional: Log publishing cycle for debugging
        # self.get_logger().debug(f'Published contact status cycle #{self.publish_count}')


def main(args=None):
    """
    Main entry point for the contact detection node.
    
    Initializes ROS2, creates the node instance, and runs the event loop
    until interrupted by user (Ctrl+C) or system shutdown.
    """
    # Initialize ROS2 Python client library
    rclpy.init(args=args)
    
    # Create the contact detection node
    node = ContactDetectionNode()
    
    try:
        # Run the node event loop
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Graceful shutdown on Ctrl+C
        node.get_logger().info('Contact detection node shutting down...')
    finally:
        # Clean up resources
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()