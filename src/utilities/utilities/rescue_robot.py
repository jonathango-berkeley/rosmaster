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
import time
import math

def clean_exit(signal, frame):
    sys.exit(0)

class RescueRobot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('rescue_robot_main')
        self.node.get_logger().info("RescueRobot initialization started.")
        super().__init__('rescue_robot_main')
        # Robot state
        self.rescue_mode = False

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Map
        self.map_data = None
        self.map_received = False  # <- log-once flag

        # Aruco Data
        self.aruco_sub = self.node.create_subscription(
            TransformStamped,
            '/aruco/transform',
            self.aruco_callback,
            10
        )
        self.aruco_queue = {}
        self.aruco_saved = []

        # Subscriptions
        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Publisher
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # HZ Store the goal pose
        self.goal_pose = None
        # Setup Magnet
        self.PIN = 32
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.PIN, GPIO.OUT, initial=GPIO.LOW)

        self.node.get_logger().info("RescueRobot ready.")

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        if not self.map_received:
            self.node.get_logger().info("Map received and updated.")
            self.map_received = True

    def aruco_callback(self, msg: TransformStamped):
        if msg.child_frame_id in self.aruco_saved:
            return

        found_location = self.get_position()
        if msg.child_frame_id in self.aruco_queue.keys:
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

    def get_position(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                "map", 'base_link', now, timeout=rclpy.duration.Duration(seconds=1.0)
            )

            return trans

        except Exception as e:
            self.node.get_logger().warn(f"TF lookup failed: {e}")

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

        self.goal_pose = pose_msg

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
        position_threshold = 0.1
        orientation_threshold = math.radians(5.0)
        stable_position_threshold = 0.01
        stable_orientation_threshold = math.radians(1.0)
        # If the goal pose has not been received yet, return False
        if self.goal_pose is None:
            self.get_logger().warn("goal pose has not been received")
            return False
    
        # Gets the current robot pose （first time）
        current_tf_1 = self.get_position()
        if current_tf_1 is None:
            self.get_logger().warn("Failed to get current pose (1st time)")
            return False
        
        # Wait 0.5 seconds to allow the robot or localization to stabilize
        time.sleep(0.5)
        
        # Get the current robot pose (second time)
        current_tf_2 = self.get_position()
        if current_tf_2 is None:
            self.get_logger().warn("Failed to get current pose (2nd time)")
            return False
        # Calculate how far the robot has moved between the first and second pose
        dx_move = current_tf_2.transform.translation.x - current_tf_1.transform.translation.x
        dy_move = current_tf_2.transform.translation.y - current_tf_1.transform.translation.y
        distance_moved = math.sqrt(dx_move**2 + dy_move**2)
        # If the movement in 0.5s exceeds the stable position threshold, the robot is still moving
        if distance_moved > stable_position_threshold:
            self.get_logger().info(
                f"The robot moved {distance_moved:.3f}m in 0.5s, still moving => not arrived!"
            )
            return False
        
        q1 = [
            current_tf_1.transform.rotation.x,
            current_tf_1.transform.rotation.y,
            current_tf_1.transform.rotation.z,
            current_tf_1.transform.rotation.w
        ]
        q2 = [
            current_tf_2.transform.rotation.x,
            current_tf_2.transform.rotation.y,
            current_tf_2.transform.rotation.z,
            current_tf_2.transform.rotation.w
        ]
        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        # Compute the relative rotation by multiplying the inverse of r1 with r2
        relative_rotation_r1_r2 = r1.inv() * r2
        angle_diff_r1_r2 = relative_rotation_r1_r2.magnitude()  
        # If the rotation in 0.5s exceeds the stable orientation threshold, the robot is still rotating
        if angle_diff_r1_r2 > stable_orientation_threshold:
            deg_12 = math.degrees(angle_diff_r1_r2)
            self.get_logger().info(
                f"The robot rotated {deg_12:.2f}° in 0.5 s, still rotating => not arrived!"
            )
            return False
        # Calculate the distance from the second pose to the goal
        dx_goal = current_tf_2.transform.translation.x - self.goal_pose.pose.position.x
        dy_goal = current_tf_2.transform.translation.y - self.goal_pose.pose.position.y
        distance_to_goal = math.sqrt(dx_goal**2 + dy_goal**2)
        # If the distance to the goal is greater than the threshold, it's not arrived yet
        if distance_to_goal > position_threshold:
            self.get_logger().info(
                f"Distance to goal: {distance_to_goal:.3f} m, not arrived yet!"
            )
            return False


        q_goal = [
            self.goal_pose.pose.orientation.x,
            self.goal_pose.pose.orientation.y,
            self.goal_pose.pose.orientation.z,
            self.goal_pose.pose.orientation.w
        ]
        r_goal = R.from_quat(q_goal)
        # Compute the relative rotation by multiplying the inverse of r2 with goal
        relative_rotation_r2_goal = r2.inv() * r_goal
        angle_diff_r2_goal = relative_rotation_r2_goal.magnitude()
        # If the orientation difference to the goal is above the threshold, it's not arrived yet
        if angle_diff_r2_goal > orientation_threshold:
            deg_2g = math.degrees(angle_diff_r2_goal)
            self.get_logger().info(
                f"Orientation difference to goal: {deg_2g:.2f}°, not arrived yet!"
            )
            return False

        # If all checks pass, log success and return True
        self.get_logger().info("Arrived at target position and orientation.")
        return True
        
       

    def search_and_rescue(self):
        pass

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
    robot.spin()

if __name__ == '__main__':
    main()
