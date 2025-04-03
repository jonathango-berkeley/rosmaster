#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

#waypoints
A = [0, 1.5239]
B = [1.5239, 1.5239]
C = [1.5239, 0]
D = [0.762, 0.762]

class ExploreHard(Node):
    def __init__(self):
        super().__init__('explore_waypoint')
        
        #subscriber
        self.subscription = self.create_subscription(Bool, '/next_waypoint', self.publish_waypoint, 10)
        
        #publisher
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        self.last_waypoint = None    #last waypoint
        
        self.get_logger().info('Node for exploration waypoints is initialized!')


    def publish_waypoint(self, sub_msg):
            pub_msg = PoseStamped()
            if sub_msg.data == True:
                if last_waypoint == None:
                    waypoint = A
                    last_waypoint = waypoint
                    #publish...
                else:
                    waypoint += 1    #make it work
                    last_waypoint = waypoint
                    #publish...

def main(args=None):
    rclpy.init(args=args)
    node = ExploreHard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
