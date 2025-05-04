#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation as R

from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import OccupancyGrid

import sys
import signal
import Hobot.GPIO as GPIO
import threading

import math
import time

import PyKDL

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
        self.aruco_queue = {}
        self.aruco_saved = []

        self.aruco_sub = self.node.create_subscription(
            TransformStamped,
            '/aruco/transform',
            self.aruco_callback,
            10
        )

        self.map_data = None
        self.map_received = False
        
        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.current_position = None

        self.pos_sub = self.node.create_subscription(
            TransformStamped,
            '/current_position',
            self.pos_callback,
            10
        )

        # Publisher
        self.pose_publisher = self.node.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.vel_publisher = self.node.create_publisher(
            Twist,
            "/cmd_vel",
            5
        )

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()

        while True:
            if self.current_position:
                self.origin = self.current_position
                break
            else:
                self.node.get_logger().warning("waiting for origin")

        # Robot Controls Part
        self.LineTolerance = 0.05
        self.RotationTolerance = math.radians(5)

        self.Linear = 0.2
        self.Angular = 0.5

        self.proportional_forward = 0.01 # Tweak P-controller
        self.proportional_spin = 0.01 # Tweak P-controller

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
    
    def get_position(self):
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
        if isinstance(pose, TransformStamped):
            pose = self._trans_to_pose(pose)

        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        x2 = self.current_position.pose.position.x
        y2 = self.current_position.pose.position.y
        angle = math.atan2(y2 - y1, x2 - x1)

        q = quaternion_from_euler(0, 0, angle)

        direction_pose = PoseStamped()
        direction_pose.header.frame_id = "map"
        direction_pose.header.stamp = rclpy.clock.Clock().now().to_msg()  # current time
        direction_pose.pose.position.x = x1
        direction_pose.pose.position.y = y1
        direction_pose.pose.position.z = 0.0
        direction_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        while self._spin(direction_pose):
            self.node.get_logger().info("Spinning...")

        while self._forward(pose):
            self.node.get_logger().info("Moving Forward...")

        while self._spin(pose):
            self.node.get_logger().info("Spinning...")
    
    def trans_to_pose(transform: TransformStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header = transform.header  # copy frame_id and timestamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _forward(self,target_position):
        position = Point()
        position.x = self.get_position().transform.translation.x
        position.y = self.get_position().transform.translation.y

        target = Point()
        target.x = target_position.transform.translation.x
        target.y = target_position.transform.translation.y

        move_cmd = Twist()
        distance = math.sqrt(pow((self.position.x - target.x), 2) + pow((self.position.y - target.y), 2))
        move_cmd.linear.x = min(self.Linear, distance*self.proportional_forward)
        if abs(distance) < self.LineTolerance: 
            self.vel_publisher.publish(Twist())
            return False
        else:
            self.vel_publisher.publish(move_cmd)
        return True

    def _spin(self,target_position):
        target_angle = self._get_yaw(target_position)
        current_angle = self._get_yaw(self.current_position)
        error = target_angle - current_angle
        move_cmd = Twist()
        move_cmd.angular.z = math.copysign(math.min(self.Angular, error*self.proportional_spin), error)
        if abs(error) < self.RotationTolerance:
            self.vel_publisher.publish(Twist())
            return False
        else:
            self.vel_publisher.publish(move_cmd)
        return True
    
    def _get_yaw(self, position):
        cacl_rot = PyKDL.Rotation.Quaternion(position.transform.rotation.x, position.transform.rotation.y,
                                                position.transform.rotation.z, position.transform.rotation.w
                                                )
        angle_rot = cacl_rot.GetRPY()[2]
    
        return angle_rot

    def switch_magnet(self, state):
        if state:
            GPIO.output(self.PIN, GPIO.HIGH)
            self.get_logger().info("Set Magnet to ON")
        else:
            GPIO.output(self.PIN, GPIO.LOW)
            self.get_logger().info("Set Magnet to OFF")

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

    while True:
        if robot.aruco_queue:
            for key in robot.aruco_queue:
                robot.run_robot(robot.aruco_queue[key]['location'])
                break

            input("wait")

            robot.run_robot(robot.origin)
            
            input('wait(2)')

            return

        time.sleep(1)

if __name__ == '__main__':
    main()
