#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

#global parameters
PIN = 32

class MagnetController(Node):
    def __init__(self):
        super().__init__('magnet_controller')
        
        self.last_state = None    #last state of magnet (on/off - True/False)
        
        #GPIO setup
        GPIO.setwarnings(False)    #GPIO warnings are not displayed
        GPIO.setmode(GPIO.BOARD)    #setting mode such that PIN number can be used
        GPIO.setup(PIN, GPIO.OUT, initial=GPIO.LOW)    #setting PIN as output
        
        #subscriber
        self.subscription = self.create_subscription(Bool, '/object_detected', self.magnet_callback, 10)    #msg type? topic name?
        self.subscription
        
        self.get_logger().info("Magnet controller node is initialized! Magnet off!")

    def clean_exit(self):
        GPIO.cleanup()
        self.get_logger().info("Magnet controller node is shutdown!")
        rclpy.shutdown()
        sys.exit(0)
        
    def magnet_callback(self, msg):
        if msg.data == self.last_state:
            return
        else:
            self.last_state = msg.data
            if msg.data == True:
                GPIO.output(PIN, GPIO.HIGH)
                self.get_logger().info("Magnet on!")
            elif msg.data == False:
                GPIO.output(PIN, GPIO.LOW)
                self.get_logger().info("Magnet off!")
            else:
                self.get_logger().warn("Invalid msg!")

def main(args=None):
    rclpy.init(args=args)
    node = MagnetController()
    
    signal.signal(signal.SIGINT, lambda sig, frame: node.clean_exit())
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.clean_exit()

if __name__=='__main__':
    main()
