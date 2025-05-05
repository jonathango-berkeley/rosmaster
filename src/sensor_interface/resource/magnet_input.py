#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO

#global parameters
PIN = 32

def clean_exit(signal, frame):
    sys.exit(0)

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
