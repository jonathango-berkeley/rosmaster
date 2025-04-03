#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

#waypoints
waypoints = {
    "base": [0, 0],
    "A": [0, 1.5239],
    "B": [1.5239, 1.5239],
    "C": [1.5239, 0],
    "D": [0.762, 0.762]
}

class ExploreHard(Node):
    def __init__(self):
        super().__init__('explore_waypoint')
        
        #subscriber
        self.subscription = self.create_subscription(Bool, '/next_waypoint', self.publish_waypoint, 10)
        
        #publisher
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.curr_waypoint = None    #current waypoint
        self.last_waypoint = None    #last waypoint
        
        self.get_logger().info('Node for exploration waypoints is initialized!')


    def publish_waypoint(self, sub_msg):
        if sub_msg.data:    #goes in if msg is true
            pub_msg = PoseStamped()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "map"

            if self.last_waypoint is None:
                self.last_waypoint = "A"
            else:
                next_key = chr(ord(self.last_waypoint) + 1)    #next unicode character
                if next_key in waypoints:    #test if key is in waypoints
                    self.last_waypoint = next_key
                else:
                    self.get_logger().warn("No more waypoints!")
                    return
            
            #set new waypoint
            self.curr_waypoint = waypoints[self.last_waypoint]
            pub_msg.pose.position.x = self.curr_waypoint[0]
            pub_msg.pose.position.y = self.curr_waypoint[1]

            pub_msg.pose.orientation.w = 1.0  #orientation?
            
            #publish
            self.publisher.publish(pub_msg)
            self.get_logger().info(f'Published new waypoint: {self.last_waypoint}:  {self.curr_waypoint}')    

def main(args=None):
    rclpy.init(args=args)
    node = ExploreHard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        self.get_logger().info("Node for exploration waypoints is shutdown!")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
