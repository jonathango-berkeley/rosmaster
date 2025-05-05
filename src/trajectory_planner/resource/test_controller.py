#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
import math
import threading
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resource')))
import a_star_Gutsav_v4 as a_star

class GoalFollower(Node):
    def __init__(self):
        super().__init__('goal_follower')

        #sub
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        #pub
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.pose_sub = self.create_subscription(TransformStamped, '/current_position', self.pose_callback, 10)

        #self.timer = self.create_timer(0.1, self.stepper)

        self.current_position = None  # Now a TransformStamped
        self.goal_pose = None
        self.map_data = None
        self.state = 'idle'  # 'rotate', 'forward', 'align', 'done'
        self.move_indi = False    #movement indicator for goal_pose (False = not moving)
        #self.move_indi2 = False    #movement indicator for interm_goal (False = not moving)
        self.new_goal = False
        self.count_interm_wayp = 0
        self.interm_wayp = []
        self.interm_goal = None

        self.linear_speed = 0.02
        self.angular_speed = 0.1
        self.position_tolerance = 0.1
        self.angle_tolerance = math.radians(2)

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        self.get_logger().info("Controller active.")

    def map_callback(self, map_msg):
        self.get_logger().info("Received a map message.")
        self.map_data = map_msg

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.new_goal = True
        self.state = 'rotate'
        self.get_logger().info("Received new goal.")

    def pose_callback(self, msg):
        self.current_position = msg
        self.get_logger().info("Received current_position.")  # Optional debug

    def control_loop(self):

        # Get current and goal positions
        x = self.current_position.transform.translation.x
        y = self.current_position.transform.translation.y
        gx = self.interm_goal.pose.position.x
        gy = self.interm_goal.pose.position.y

        # Get current yaw
        q = self.current_position.transform.rotation
        yaw = self.yaw_from_quaternion(q)

        # Compute goal direction
        dx = gx - x
        dy = gy - y
        distance = math.hypot(dx, dy)
        goal_theta = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(goal_theta - yaw)

        twist = Twist()

        if self.state == 'rotate':
            if abs(angle_diff) > self.angle_tolerance:
                twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                self.get_logger().info(f"Outside tolerance. angle_diff: {angle_diff} > {self.angle_tolerance}")
            else:
                twist = Twist()
                self.get_logger().info("Stopped!")
                self.state = 'forward'
                self.get_logger().info("Rotation done. Driving forward.")
        
        elif self.state == 'forward':
            if distance > self.position_tolerance:
                twist.linear.x = self.linear_speed
                self.get_logger().info(f"Outside tolerance. distance: {distance}")
            else:
                twist = Twist()
                self.get_logger().info("Stopped!")
                self.state = 'align'
                self.get_logger().info("Position reached. Aligning to goal orientation.")
        
        elif self.state == 'align':
            goal_q = self.interm_goal.pose.orientation
            goal_yaw = self.yaw_from_quaternion(goal_q)
            final_diff = self.normalize_angle(goal_yaw - yaw)

            if abs(final_diff) > self.angle_tolerance:
                twist.angular.z = self.angular_speed if final_diff > 0 else -self.angular_speed
                self.get_logger().info(f"Outside tolerance. final_diff: {final_diff}")
            else:
                twist = Twist()
                self.get_logger().info("Stopped!")
                self.state = 'done'
                self.get_logger().info("Goal reached and orientation aligned.")

        elif self.state == 'done':
            twist = Twist()
            #self.move_indi2 = False
            self.get_logger().info("Done!")

        self.publisher_.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
            #print("normalized")
        while angle < -math.pi:
            angle += 2 * math.pi
            #print("normalized")
        return angle

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stepper(self):
        if self.current_position is None or self.goal_pose is None or self.map_data is None or self.new_goal is False:
            return

        self.new_goal = False
        self.count_interm_wayp = 0
        
        #transform goal_waypoint and prev_waypoint into map coordinates (cells)
        res = self.map_data.info.resolution
        orix = self.map_data.info.origin.position.x
        oriy = self.map_data.info.origin.position.y
        goal_x = int((self.goal_pose.pose.position.x-orix)/res)
        goal_y = int((self.goal_pose.pose.position.y-oriy)/res)
        prev_x = int((self.current_position.transform.translation.x-orix)/res)
        prev_y = int((self.current_position.transform.translation.y-oriy)/res)
        
        #plan path
        #a_star.plot(self.map_data, [prev_x, prev_y], [goal_x, goal_y])
        trajectory = a_star.a_star(self.map_data, [prev_x, prev_y], [goal_x, goal_y])
            
        if trajectory == None:
            self.get_logger().info("No path found!")
            return
            
        #extracting cell coordinates and transform back to real world coordinates
        for i in range(len(trajectory)):
            point = PoseStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = "map"

            point.pose.position.x = (trajectory[i].y * res)+orix
            point.pose.position.y = (trajectory[i].x * res)+oriy

            if i == (len(trajectory) - 1):
                dx = self.goal_pose.pose.position.x - point.pose.position.x
                dy = self.goal_pose.pose.position.y - point.pose.position.y
            else:
                dx = ((trajectory[(i+1)].y * res)+orix) - point.pose.position.x
                dy = ((trajectory[(i+1)].x * res)+oriy) - point.pose.position.y
            theta = math.atan2(dy, dx)

            point.pose.orientation.z = math.sin(theta / 2.0)
            point.pose.orientation.w = math.cos(theta / 2.0)

            self.interm_wayp.append(point)    #[(trajectory[i].y * res)+orix, (trajectory[i].x * res)+oriy])
                
        self.interm_wayp.append(self.goal_pose)

        #self.waypoint_coords = self.interm_wayp[self.count_interm_wayp]

        self.move_indi = True
        
        #loop to move
        while self.move_indi is True:

            if (self.count_interm_wayp) > (len(self.interm_wayp)-1):
                self.interm_goal = self.goal_pose
                self.move_indi = False
            else:
                self.interm_goal = self.interm_wayp[self.count_interm_wayp]
            
            #self.get_logger().info(f"interm_goal: {self.interm_goal}")
            #self.move_indi2 = True
            self.state = 'rotate'

            while self.state != 'done':
                self.control_loop()
                time.sleep(0.1)

            self.count_interm_wayp += 1

        self.interm_wayp.clear()

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)
        self.get_logger().info("Stopped!")

def main(args=None):
    rclpy.init(args=args)
    node = GoalFollower()
    try:
        while True:
            node.stepper()
    except KeyboardInterrupt:
        print("Shuting down...")
    finally:
        node.stop()
        print("Shut down!")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()