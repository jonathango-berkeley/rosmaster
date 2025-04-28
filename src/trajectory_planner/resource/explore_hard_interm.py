#!/usr/bin/env python3
import sys
import os
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resource')))
import a_star_Gutsav_v4 as a_star

class ExploreHard(Node):
    def __init__(self):
        super().__init__('explore_waypoint')
        
        #subscriber
        self.subscriber = self.create_subscription(Bool, '/next_waypoint', self.publish_waypoint, 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        
        #publisher
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.waypoints = [
            [0.0, 0.0],
            [1.5239, 0.0],
            [1.5239, -1.5239],
            [0.0, -1.5239],
            [0.762, -0.762]
        ]
        self.num_interm_wayp = 2    #total number of intermediate waypoints (needs tuning)
        self.prev_waypoint = 0    #previous waypoint list position
        self.waypoint_coords = None    #coordinates of the waypoint we currently want to move to
        self.goal_waypoint = 1    #goal waypoint list position
        self.count_interm_wayp = 0    #count of intermediate waypoint
        self.interm_wayp = []    #list of intermediate waypoints
        self.map_data = None    #map
        
        self.get_logger().info('Node for exploration waypoints is initialized!')


    def publish_waypoint(self, sub_msg):
            
        #set goal waypoint
        if self.waypoints[self.goal_waypoint] != self.waypoint_coords:
            self.count_interm_wayp += 1
        else:
            if self.goal_waypoint == 0:
                self.get_logger().info("Exploration finished!")
                return None    #ends exploration (the script could also be restarted here)
            else:
                next_key = self.goal_waypoint + 1
            
            self.interm_wayp.clear()
            
            if next_key < len(self.waypoints):    #test if key is in waypoints
                self.prev_waypoint = self.goal_waypoint
                self.goal_waypoint = next_key
            else:
                self.prev_waypoint = self.goal_waypoint
                self.goal_waypoint = 0
                self.get_logger().info("Return to the base!")
            
        #set waypoint (intermediate or goal)
        if not self.interm_wayp:

            self.count_interm_wayp = 0
            
            #transform goal_waypoint and prev_waypoint into map coordinates (cells)
            res = self.map_data.info.resolution
            orix = self.map_data.info.origin.position.x
            oriy = self.map_data.info.origin.position.y
            goal_x = int((self.waypoints[self.goal_waypoint][0]+orix)/res)
            goal_y = int((self.waypoints[self.goal_waypoint][1]+oriy)/res)
            prev_x = int((self.waypoints[self.prev_waypoint][0]+orix)/res)
            prev_y = int((self.waypoints[self.prev_waypoint][1]+oriy)/res)
            
            #plan path
            #a_star.plot(self.map_data, [prev_x, prev_y], [goal_x, goal_y])
            trajectory = a_star.a_star(self.map_data, [prev_x, prev_y], [goal_x, goal_y])
                
            #extract intermediate waypoints
            k, m = divmod(len(trajectory), self.num_interm_wayp + 1)
            parts = [trajectory[i * k + min(i, m):(i + 1) * k + min(i + 1, m)] for i in range(self.num_interm_wayp + 1)]    #dividing the list of waypoints
            waypoints_map = [parts[i][-1] for i in range(self.num_interm_wayp)]    #extracting intermediate waypoints
                
            #extracting cell coordinates and transform back to real world coordinates
            for i in range(len(waypoints_map)):
                self.interm_wayp.append([(waypoints_map[i].y * res)-oriy, (waypoints_map[i].x * res)-orix])
                
            self.interm_wayp.append(self.waypoints[self.goal_waypoint])

        self.waypoint_coords = self.interm_wayp[self.count_interm_wayp]
        
        pub_msg = PoseStamped()
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        pub_msg.header.frame_id = "map"
         
        pub_msg.pose.position.x = self.waypoint_coords[0]
        pub_msg.pose.position.y = self.waypoint_coords[1]
            
        #calculate orientation (facing towards the center)
        if self.waypoint_coords != self.waypoints[4]:
            dx = self.waypoints[4][0] - self.waypoint_coords[0]
            dy = self.waypoints[4][1] - self.waypoint_coords[1]
        else:
            dx = self.waypoints[0][0] - self.waypoint_coords[0]
            dy = self.waypoints[0][1] - self.waypoint_coords[1]
        theta = math.atan2(dy, dx)

        pub_msg.pose.orientation.z = math.sin(theta / 2.0)  #orientation?
        pub_msg.pose.orientation.w = math.cos(theta / 2.0)  #orientation?
            
        #publish
        self.publisher.publish(pub_msg)
        self.get_logger().info(f'Published new waypoint: {self.waypoint_coords}')    
        
    def map_callback(self, map_msg):
        self.get_logger().info("Received a map message.")
        self.map_data = map_msg

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
