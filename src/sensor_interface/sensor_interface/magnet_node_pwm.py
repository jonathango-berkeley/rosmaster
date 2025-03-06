#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32

#global parameters
PIN = 32
PWM_freq = 1000

class MagnetController(Node):
    def __init__(self):
        super().__init__('magnet_pwm_controller')
        
        self.last_state = 0    #last duty cycle of magnet (%)
        
        #GPIO setup
        GPIO.setwarnings(False)    #GPIO warnings are not displayed
        GPIO.setmode(GPIO.BOARD)    #setting mode such that PIN number can be used
        
        self.pwm = GPIO.PWM(PIN, PWM_freq)
        self.pwm.start(0)
        
        #subscriber
        self.subscription = self.create_subscription(Float32, '/magnet_pwm_control', self.magnet_callback, 10)    #msg type? topic name?
        self.subscription
        
        self.get_logger().info("Magnet controller node (PWM) is initialized! Duty cycle 0%!")

    def clean_exit(self):
        self.pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("Magnet controller node is shutdown!")
        rclpy.shutdown()
        sys.exit(0)
        
    def magnet_callback(self, msg):
        if msg.data == self.last_state:
            return
        else:
            duty_cycle = max(0.0, min(100.0, msg.data))
            if duty_cycle != msg.data:
                self.get_logger().warn("Msg value out of range!")
            
            self.last_state = duty_cycle
            self.pwm.ChangeDutyCycle(duty_cycle)
            
            self.get_logger().info(f"Duty cycle {duty_cycle}%!")

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
