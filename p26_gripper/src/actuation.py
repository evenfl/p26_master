#!/usr/bin/env python3.7

import time
import sys
import numpy as np
import RPi.GPIO as GPIO
from pyfirmata import Arduino, util
import pigpio as io

# Setup for communication with Arduino
board = Arduino('/dev/ttyACM0') # Define device connection
it = util.Iterator(board)       # Receive data into iterator
it.start()
time.sleep(0.1)                 # Wait for iterator to start
current_sense = board.analog[0] # Define analog pin 0 as current sensor
arm_pos = board.analog[5]       # Define analog pin 5 as potmeter

# Setup for GPIO on Raspberry
GPIO.setwarnings(False) # Disable warnings from GPIO
GPIO.setmode(GPIO.BOARD) # Use onboard GPIO on Raspberry
pi = io.pi()
pi.set_mode(22, io.OUTPUT)
pi.set_PWM_frequency(22, 20000) # Set pin 15 (BCM 22) as motor PWM output at 20kHz
GPIO.setup(11, GPIO.OUT) # Set pin 11 as output
GPIO.setup(13, GPIO.OUT) # Set pin 13 as output

max_opening_current = 0.55 # Maximum current value for motor at opening
max_closing_current = 2.4 # Maximum current value for motot at closing
max_opening_pos = 0.53 # Maximum opening position value
min_closing_pos_200 = 0.47 # Minimum closing position value for 200mm cylinder
min_closing_pos_260 = 0.485 # Minimum closing position value for 260mm cylinder


def CCW(speed):
    GPIO.output(11, GPIO.HIGH)
    GPIO.output(13, GPIO.LOW)
    pi.set_PWM_dutycycle(22, speed/100 * 255)   

def CW(speed):
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.HIGH)
    pi.set_PWM_dutycycle(22, speed/100 * 255)    

def current_average(): # Finds and calculates average/filtered current
    current_it = 0
    measurement = 40
    while current_sense.read() == None: # Passes through initial None value
        pass
    for current_count in np.arange(1, measurement, 1): # 40 measurements
        current_it += current_sense.read()
        time.sleep(0.1/measurement) # Time between each measurement
 
    current_avg = np.round(5/0.13*current_it/measurement, 5) # Average of 40 measurement made to estimated ampere, rounded to 5 decimals
    return current_avg

def hold(): # Holds torque at the motor by braking to ground
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.LOW)
    pi.set_PWM_dutycycle(22, 0)

def extend(): # Opening of the arms
    print("Opening gripper...")
    current_sense.enable_reporting() # Start reporting from current_sense and arm_pos
    arm_pos.enable_reporting()
    extended = 0
    current_avg = 0
    current_max_count = 0
    arm_pos_max = 0

    while current_sense.read() == None: # Passes through initial None values
        pass
    while arm_pos.read() == None:
        pass

    if arm_pos.read() >= max_opening_pos: # Checks if arms alrady in open position
        hold()
        extended = 1
        print("Gripper already open")

    while extended == 0: # Runs while arms are not in open position
        CW(20)

        while current_max_count < 3 and arm_pos_max < 2: # Runs while position and current is within threshold values
            current_avg = current_average()
            arm_position = arm_pos.read()      

            if current_avg > max_opening_current:
                current_max_count += 1

            if arm_position > max_opening_pos:
                arm_pos_max += 1
        
            print(current_avg, ", ", arm_position)
        extended = 1

    hold()
    current_sense.disable_reporting() # Stops reporting from inputs
    arm_pos.disable_reporting()

def retract(cylinder_size):
    print("Closing gripper...")
    current_sense.enable_reporting() # Start reporting from inputs
    arm_pos.enable_reporting()
    CCW(30)
    current_avg = 0
    current_max_count = 0
    arm_pos_min_count = 0

    if cylinder_size == 260:
        arm_pos_min = min_closing_pos_260
    
    elif cylinder_size == 200:
        arm_pos_min = min_closing_pos_200

    while current_sense.read() == None: # Passes through initial None value
        pass
    while arm_pos.read() == None:
        pass

    while current_max_count < 3 and arm_pos_min_count < 2:
        if current_avg > max_closing_current:
            current_max_count += 1
    
        arm_position = arm_pos.read()
        if arm_position < arm_pos_min:
            arm_pos_min_count += 1
        current_avg = current_average()
        print(current_avg, ", ", arm_position)
    CCW(9) # Hold motor still with sufficient strength 
    current_sense.disable_reporting() # Stop reporting from sensors
    arm_pos.disable_reporting()

# try:
#     while True:
#         extend()
#         time.sleep(2)
#         retract(260)
#         time.sleep(100)

# except:
#     hold()
#     print("An error occured, stopping motor")
