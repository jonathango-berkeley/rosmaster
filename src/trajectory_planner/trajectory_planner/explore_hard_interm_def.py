#!/usr/bin/env python3
import os    #add this to the imports

#####
#change to wherever a_star_Gutsav_v4 is located
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resource')))
#####
import a_star_Gutsav_v4 as a_star    #add this to the imports

#gobal parameters    #add these also
waypoints = {
    "base": [0.0, 0.0],
    "A": [0.0, 1.5239],
    "B": [1.5239, 1.5239],
    "C": [1.5239, 0.0],
    "D": [0.762, 0.762]
}
num_interm_wayp = 2    #total number of intermediate waypoints (needs tuning)

        #####
        #add this to the __init__
        self.prev_waypoint = None    #previous waypoint
        self.curr_waypoint = None    #current waypoint
        self.goal_waypoint = None    #goal waypoint
        self.count_interm_wayp = 0    #count of intermediate waypoint
        self.interm_wayp = []    #list of intermediate waypoints
        #####

    #####
    #add this function to the masterscript to get next waypoint, if you call next_explore_waypoint()
    def next_explore_waypoint(self):
        if True:
            
            #set goal waypoint
            if self.goal_waypoint is None:
                self.prev_waypoint = "base"
                self.goal_waypoint = "A"
            elif waypoints[self.goal_waypoint] != self.curr_waypoint:
                self.count_interm_wayp += 1
            else:
                if self.goal_waypoint == "base":
                    next_key = "A"    #restarts exploration (the script could also be stopped here)
                else:
                    next_key = chr(ord(self.goal_waypoint) + 1)    #next unicode character
                self.count_interm_wayp = 0
                self.interm_wayp.clear()
                if next_key in waypoints:    #test if key is in waypoints
                    self.prev_waypoint = self.goal_waypoint
                    self.goal_waypoint = next_key
                else:
                    self.prev_waypoint = self.goal_waypoint
                    self.goal_waypoint = "base"
                    self.get_logger().info("Return to the base!")
            
            #set waypoint (intermediate or goal)
            if not self.interm_wayp:
                
                #transform goal_waypoint and prev_waypoint into map coordinates (cells)
                res = self.map_data.info.resolution
                goal_x = int(waypoints[self.goal_waypoint][0]/res)
                goal_y = int(waypoints[self.goal_waypoint][1]/res)
                prev_x = int(waypoints[self.prev_waypoint][0]/res)
                prev_y = int(waypoints[self.prev_waypoint][1]/res)
                
                #plan path
                #a_star.plot(self.map_data, [prev_x, prev_y], [goal_x, goal_y])
                trajectory = a_star.a_star(self.map_data, [prev_x, prev_y], [goal_x, goal_y])
                
                #extract intermediate waypoints
                k, m = divmod(len(trajectory), num_interm_wayp + 1)
                parts = [trajectory[i * k + min(i, m):(i + 1) * k + min(i + 1, m)] for i in range(num_interm_wayp + 1)]    #dividing the list of waypoints
                waypoints_map = [parts[i][-1] for i in range(num_interm_wayp)]    #extracting intermediate waypoints
                
                #extracting cell coordinates and transform back to real world coordinates
                for i in range(len(waypoints_map)):
                    self.interm_wayp.append([waypoints_map[i].y * res, waypoints_map[i].x * res])
                
                self.interm_wayp.append(waypoints[self.goal_waypoint])
                self.curr_waypoint = self.interm_wayp[self.count_interm_wayp]
            else:
                self.curr_waypoint = self.interm_wayp[self.count_interm_wayp]

            pub_msg = PoseStamped()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "map"
            
            pub_msg.pose.position.x = float(self.curr_waypoint[0])
            pub_msg.pose.position.y = float(self.curr_waypoint[1])
            
            #calculate orientation (facing towards the center)
            if self.curr_waypoint != waypoints["D"]:
                dx = waypoints["D"][0] - self.curr_waypoint[0]
                dy = waypoints["D"][1] - self.curr_waypoint[1]
            else:
                dx = waypoints["base"][0] - self.curr_waypoint[0]
                dy = waypoints["base"][1] - self.curr_waypoint[1]
            theta = math.atan2(dy, dx)

            pub_msg.pose.orientation.z = math.sin(theta / 2.0)  #orientation?
            pub_msg.pose.orientation.w = math.cos(theta / 2.0)  #orientation?
            
            return pub_msg       #returns the next waypoint as PoseStamped in map coordinates (can be published directly)
