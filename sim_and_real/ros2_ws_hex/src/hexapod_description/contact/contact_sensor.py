#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ros_gz_interfaces.msg import Contacts


class ContactDetectionNode(Node):
    def __init__(self):
        super().__init__('contact_detection_node')
        
        # Słowniki do przechowywania subskrypcji, publisherów i flag dla każdej nogi
        self.contact_subscribers = {}
        self.contact_publishers = {}
        self.received_flags = {}
        self.message_counts = {}
        
        # Konfiguracja dla każdej nogi (1-6)
        for leg_num in range(1, 7):
            # Topic Gazebo dla sensora kontaktu
            gazebo_topic = f'/world/empty/model/hexapod/link/link5_{leg_num}/sensor/sensor_contact_{leg_num}/contact'
            
            # Topic ROS do publikowania statusu
            ros_topic = f'/hexapod/leg{leg_num}/contact_status'
            
            # Tworzenie subskrypcji
            self.contact_subscribers[leg_num] = self.create_subscription(
                Contacts,
                gazebo_topic,
                lambda msg, ln=leg_num: self.contact_callback(msg, ln),
                10
            )
            
            # Tworzenie publishera
            self.contact_publishers[leg_num] = self.create_publisher(
                Bool,
                ros_topic,
                10
            )
            
            # Inicjalizacja flag i liczników
            self.received_flags[leg_num] = False
            self.message_counts[leg_num] = 0
        
        # Timer do regularnego publikowania (co 100ms)
        self.publish_timer = self.create_timer(0.1, self.publish_all_status)
        
        # Licznik publikacji
        self.publish_count = 0
        
    def contact_callback(self, msg, leg_number):
        """Callback wywoływany gdy przyjdzie wiadomość z Gazebo dla danej nogi"""
        self.message_counts[leg_number] += 1
        self.received_flags[leg_number] = True
        
    def publish_all_status(self):
        """Publikuje status dla wszystkich nóg co określony czas"""
        self.publish_count += 1
        
        # Publikuj status dla każdej nogi
        for leg_num in range(1, 7):
            # Tworzymy wiadomość Bool
            msg = Bool()
            msg.data = self.received_flags[leg_num]
            
            # Publikujemy
            self.contact_publishers[leg_num].publish(msg)
            
            # Resetujemy flagę po publikacji
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