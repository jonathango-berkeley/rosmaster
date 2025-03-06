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
        self.subscription = self.create_subscription(Bool, '/magnet_control', self.magnet_callback, 10)    #msg type? topic name?
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
"""
def main():
    GPIO.setwarnings(False)    #GPIO warnings are not displayed
    GPIO.setmode(GPIO.BOARD)    #setting mode such that PIN number can be used
    GPIO.setup(PIN, GPIO.OUT, initial=GPIO.LOW)    #setting PIN as output
    try:
        while True:
            state = int(input("Turn magnet on/off ([1]/[0]): "))
            if state == 1:
                GPIO.output(PIN, GPIO.HIGH)
                print("Magnet on!")
            elif state == 0:
                GPIO.output(PIN, GPIO.LOW)
                print("Magnet off!")
            else:
                raise Exception("Invalid input!")
    finally:
        print('\r'+"Shut down!"+" "*22)
        GPIO.cleanup()

if __name__=='__main__':
    signal.signal(signal.SIGINT, clean_exit)
    main()
"""
