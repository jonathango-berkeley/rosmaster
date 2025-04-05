#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid

import resource.a_star_Gutsav_v4 as a_star

#gobal parameters
waypoints = {
    "base": [0.0, 0.0],
    "A": [0.0, 1.5239],
    "B": [1.5239, 1.5239],
    "C": [1.5239, 0.0],
    "D": [0.762, 0.762]
}
num_interm_wayp = 2    #total number of intermediate waypoints

class ExploreHard(Node):
    def __init__(self):
        super().__init__('explore_waypoint')
        
        #subscriber
        self.subscriber = self.create_subscription(Bool, '/next_waypoint', self.publish_waypoint, 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback)
        
        #publisher
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.prev_waypoint = None    #previous waypoint
        self.curr_waypoint = None    #current waypoint
        self.goal_waypoint = None    #goal waypoint
        self.count_interm_wayp = 0    #count of intermediate waypoint
        self.interm_wayp = []    #list of intermediate waypoints
        self.current_map = None    #map
        
        self.get_logger().info('Node for exploration waypoints is initialized!')


    def publish_waypoint(self, sub_msg):
        if sub_msg.data:    #goes in if msg is true
            
            #set goal waypoint
            if self.goal_waypoint is None:
                self.prev_waypoint = "base"
                self.goal_waypoint = "A"
            elif waypoints[self.goal_waypoint] != self.curr_waypoint:
                self.count_interm_wayp += 1
            else:
                next_key = chr(ord(self.goal_waypoint) + 1)    #next unicode character
                self.count_interm_wayp = 0
                self.interm_wayp.clear()
                if next_key in waypoints:    #test if key is in waypoints
                    self.goal_waypoint = next_key
                else:
                    self.get_logger().warn("No more waypoints!")
                    return
            
            #set waypoint (intermediate or goal)
            if not self.interm_wayp:
                
                #transform waypoints[self.goal_waypoint] and waypoints[self.prev_waypoint] into map coordinates
                
                #plan path
                trajectory = a_star.a_star(self.current_map, map_goalpoint, map_startpoint)
                
                #transform trajectory back to real world coordinates
                
                #extract intermediate waypoints
                
                self.interm_wayp.append(waypoints[self.goal_waypoint])
                self.curr_waypoint = self.interm_wayp[self.count_interm_wayp]
            else:
                self.curr_waypoint = self.interm_wayp[self.count_interm_wayp]

            pub_msg = PoseStamped()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "map"
            
            pub_msg.pose.position.x = float(self.curr_waypoint[0])
            pub_msg.pose.position.y = float(self.curr_waypoint[1])

            pub_msg.pose.orientation.w = 1.0  #orientation?
            
            #publish
            self.publisher.publish(pub_msg)
            self.get_logger().info(f'Published new waypoint: {self.curr_waypoint}')    
        
    def map_callback(self, map_msg):
        self.get_logger().info("Received a map message.")
        self.current_map = map_msg

def main(args=None):
    rclpy.init(args=args)
    node = ExploreHard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Node for exploration waypoints is shutdown!")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
