#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation as R

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid

import sys
import signal
import Hobot.GPIO as GPIO
import threading
import os

import math
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resource')))
import a_star_Gutsav_v4 as a_star

def clean_exit(signal, frame):
    sys.exit(0)

class RescueRobot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('rescue_robot_main')
        self.node.get_logger().info("RescueRobot initialization started.")

        # Robot state
        self.rescue_mode = False

        # Subscriptions
        self.aruco_sub = self.node.create_subscription(
            TransformStamped,
            '/aruco/transform',
            self.aruco_callback,
            10
        )
        self.aruco_queue = {}
        self.aruco_saved = []

        
        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.map_data = None
        self.map_received = False  # <- log-once flag

        self.pos_sub = self.node.create_subscription(
            TransformStamped,
            '/current_position',
            self.pos_callback,
            10
        )

        self.current_position = None

        # Publisher
        self.pose_publisher = self.node.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()

        # Setup Magnet
        self.PIN = 32
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.PIN, GPIO.OUT, initial=GPIO.LOW)
        
        # For exploration
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

        self.node.get_logger().info("RescueRobot ready.")

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        if not self.map_received:
            self.node.get_logger().info("Map received and updated.")
            self.map_received = True

    def aruco_callback(self, msg: TransformStamped):
        if msg.child_frame_id in self.aruco_saved:
            return

        found_location = self.get_current_position()
        if msg.child_frame_id in self.aruco_queue:
            self.aruco_queue[msg.child_frame_id]["found_location"] = found_location
            if len(self.aruco_queue[msg.child_frame_id]["last_10"]) > 10:
                self.aruco_queue[msg.child_frame_id]["last_10"].pop(0)

            self.aruco_queue[msg.child_frame_id]["last_10"].append(msg)
            self.aruco_queue[msg.child_frame_id]["location"] = self.filter_location(self.aruco_queue[msg.child_frame_id]["last_10"])
        else:
            self.aruco_queue[msg.child_frame_id] = {
                "location": msg,
                "found_location": found_location,
                "last_10": [msg]
            }
    
    def remove_object(self, child_frame_id):
        try:
            self.aruco_queue.pop(child_frame_id)
            self.aruco_saved.append(child_frame_id)
        except KeyError:
            self.node.get_logger().error(f"Key {child_frame_id} not found")

    def pos_callback(self, msg):
        self.current_position = msg
    
    def get_current_position(self):
        if self.current_position is None:
            self.node.get_logger().warn("Current position not yet received.")
            return None
        return self.current_position

    def filter_location(self, transforms):
        positions = []
        quaternions = []

        for msg in transforms:
            t = msg.transform.translation
            q = msg.transform.rotation
            positions.append([t.x, t.y, t.z])
            quaternions.append([q.x, q.y, q.z, q.w])

        # Average position
        avg_pos = np.mean(positions, axis=0)

        # Average quaternion using scipy Rotation
        avg_rot = R.from_quat(quaternions).mean().as_quat()

        # Use the last transform's header and child_frame_id
        last_msg = transforms[-1]
        filtered_msg = TransformStamped()
        filtered_msg.header.stamp = last_msg.header.stamp
        filtered_msg.header.frame_id = last_msg.header.frame_id
        filtered_msg.child_frame_id = last_msg.child_frame_id

        filtered_msg.transform.translation.x = avg_pos[0]
        filtered_msg.transform.translation.y = avg_pos[1]
        filtered_msg.transform.translation.z = avg_pos[2]
        filtered_msg.transform.rotation.x = avg_rot[0]
        filtered_msg.transform.rotation.y = avg_rot[1]
        filtered_msg.transform.rotation.z = avg_rot[2]
        filtered_msg.transform.rotation.w = avg_rot[3]

        return filtered_msg
    
    def run_robot(self, pose):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = pose.header.stamp
        pose_msg.header.frame_id = pose.header.frame_id  # e.g., "map"

        pose_msg.pose.position.x = pose.transform.translation.x
        pose_msg.pose.position.y = pose.transform.translation.y
        pose_msg.pose.position.z = pose.transform.translation.z

        pose_msg.pose.orientation.x = pose.transform.rotation.x
        pose_msg.pose.orientation.y = pose.transform.rotation.y
        pose_msg.pose.orientation.z = pose.transform.rotation.z
        pose_msg.pose.orientation.w = pose.transform.rotation.w

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info("Published PoseStamped to /goal_pose")

    def switch_magnet(self, state):
        if state:
            GPIO.output(self.PIN, GPIO.HIGH)
            self.get_logger().info("Set Magnet to ON")
        else:
            GPIO.output(self.PIN, GPIO.LOW)
            self.get_logger().info("Set Magnet to OFF")


    def is_arrived(self):
        pass

    def search_and_rescue(self):
        pass

    def get_next_waypoint(self):
        
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
            goal_x = int((self.waypoints[self.goal_waypoint][0]-orix)/res)
            goal_y = int((self.waypoints[self.goal_waypoint][1]-oriy)/res)
            prev_x = int((self.waypoints[self.prev_waypoint][0]-orix)/res)
            prev_y = int((self.waypoints[self.prev_waypoint][1]-oriy)/res)
            
            #plan path
            #a_star.plot(self.map_data, [prev_x, prev_y], [goal_x, goal_y])
            trajectory = a_star.a_star(self.map_data, [prev_x, prev_y], [goal_x, goal_y])
                
            #extract intermediate waypoints
            k, m = divmod(len(trajectory), self.num_interm_wayp + 1)
            parts = [trajectory[i * k + min(i, m):(i + 1) * k + min(i + 1, m)] for i in range(self.num_interm_wayp + 1)]    #dividing the list of waypoints
            waypoints_map = [parts[i][-1] for i in range(self.num_interm_wayp)]    #extracting intermediate waypoints
                
            #extracting cell coordinates and transform back to real world coordinates
            for i in range(len(waypoints_map)):
                self.interm_wayp.append([(waypoints_map[i].y * res)+orix, (waypoints_map[i].x * res)+oriy])
                
            self.interm_wayp.append(self.waypoints[self.goal_waypoint])

        self.waypoint_coords = self.interm_wayp[self.count_interm_wayp]
        
        pub_msg = TransformStamped()
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        pub_msg.header.frame_id = "map"
         
        pub_msg.transform.translation.x = self.waypoint_coords[0]
        pub_msg.transform.translation.y = self.waypoint_coords[1]
            
        #calculate orientation (facing towards the center)
        if self.waypoint_coords != self.waypoints[4]:
            dx = self.waypoints[4][0] - self.waypoint_coords[0]
            dy = self.waypoints[4][1] - self.waypoint_coords[1]
        else:
            dx = self.waypoints[0][0] - self.waypoint_coords[0]
            dy = self.waypoints[0][1] - self.waypoint_coords[1]
        theta = math.atan2(dy, dx)

        pub_msg.transform.rotation.z = math.sin(theta / 2.0)  #orientation?
        pub_msg.transform.rotation.w = math.cos(theta / 2.0)  #orientation?
            
        return pub_msg       #returns the next waypoint as TransformStamped in map coordinates (can be sent to run_robot directly)

    def spin(self):
        self.node.get_logger().info("Robot is running...")
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            self.node.get_logger().info("Shutting down robot...")
        finally:
            self.shutdown()

    def shutdown(self):
        GPIO.cleanup()
        self.node.get_logger().info("Cleaning up resources.")
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    signal.signal(signal.SIGINT, clean_exit)
    robot = RescueRobot()

    while True:
        if robot.current_position is not None:
            print(f"current position: {robot.current_position.transform.translation}")
        else:
            print("no position found")

        try:
            print("----- aruco queue ------")
            for key in robot.aruco_queue:
                loc = robot.aruco_queue[key]["location"]
                print(f"{key} location: {loc.transform.translation}")
        except Exception as e:
            print(f"error: {e}")

        time.sleep(1)

if __name__ == '__main__':
    main()
