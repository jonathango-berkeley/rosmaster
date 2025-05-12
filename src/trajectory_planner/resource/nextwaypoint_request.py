#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

class MagnetPublisher(Node):
    def __init__(self):
        super().__init__('nextwaypoint_request')
        
        self.bool_publisher = self.create_publisher(Bool, '/next_waypoint', 10)
        
        self.get_logger().info("Waypoint request Publisher gestartet! Warte auf Eingaben...")

    def publish_waypoint_request(self, state: bool):
        msg = Bool()
        msg.data = state
        self.bool_publisher.publish(msg)
        self.get_logger().info(f"Gesendet: Waypoint {'requested' if state else 'not requested'}")

def main(args=None):
    rclpy.init(args=args)
    node = MagnetPublisher()

    try:
        while rclpy.ok():
            mode = input("Request waypoint?")
            
            if mode:
                value = True
                node.publish_waypoint_request(value)

            else:
                print("Ungültige Auswahl!")

    except KeyboardInterrupt:
        print("Beende Magnet Publisher...")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

