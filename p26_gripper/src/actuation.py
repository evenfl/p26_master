#!/usr/bin/env python3.7

import time
import sys
import numpy as np
import RPi.GPIO as GPIO
from pyfirmata import Arduino, util
import pigpio as io
import csv



# Setup for communication with Arduino
board = Arduino('/dev/ttyACM0') # Define device connection
it = util.Iterator(board)   # Receive data into iterator
it.start()
time.sleep(0.1)
current_sense = board.analog[0] # Define analog pin 0 as current sensor
arm_pos = board.analog[5] # Define analog pin 5 as potmeter
#current_sense.enable_reporting() # Start reporting from current_sense


# Setup for GPIO on Raspberry
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) # Use onboard GPIO on Raspberry
pi = io.pi()
pi.set_mode(22, io.OUTPUT)
pi.set_PWM_frequency(22, 20000) # Set pin 22 as motor PWM output at 20000 Hz
#GPIO.setup(33, GPIO.OUT) # Set pin 33 as output
GPIO.setup(11, GPIO.OUT) # Set pin 11 as output
GPIO.setup(13, GPIO.OUT) # Set pin 13 as output

#motor = GPIO.PWM(33, 20000) # Set pin 33 as motor PWM output at 20000 Hz

max_opening_current = 0.38 # Maximum current for motor at opening
max_closing_current = 1.5 # Maximum current for motot at closing
max_opening_pos = 0.51 # Maximum opening position
min_closing_pos_200 = 0.42 # Minimum closing position for 200mm cylinder
min_closing_pos_260 = 0.47 # Minimum closing position for 260mm cylinder

def CW(speed):
    GPIO.output(11, GPIO.HIGH)
    GPIO.output(13, GPIO.LOW)
    pi.set_PWM_dutycycle(22, speed/100 * 255)   


def CCW(speed):
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.HIGH)
    pi.set_PWM_dutycycle(22, speed/100 * 255)   
    

def sleep(sec):
    for sleep in np.arange(0, sec, 0.2):
        current_average()
        time.sleep(0.2)
    current_sense.disable_reporting()    

def current_average(): # Finds and calculates average current
    current_it = 0
    while current_sense.read() == None: # Passes through initial None value
        pass
    for current_count in np.arange(1, 40, 1): # 40 measurements
        current_it += current_sense.read()
        time.sleep(0.1/40) # Time between each measurement
 
    current_avg = np.round(5/0.13*current_it/40, 5) # Average of 40 measurement, rounded to 5 decimals
    return current_avg

def hold(): # Holds torque at the motor by brake to ground
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.LOW)
    pi.set_PWM_dutycycle(22, 0)

def extend():
    print("Opening gripper...")
    current_sense.enable_reporting() # Start reporting from current_sense
    arm_pos.enable_reporting()
    CW(20)
    current_avg = 0
    current_max_count = 0
    arm_pos_max = 0
    
    while current_sense.read() == None: # Passes through initial None value
        pass
    while arm_pos.read() == None:
        pass

    while current_max_count < 3 and arm_pos_max < 2:
        if current_avg > max_opening_current:
            current_max_count += 1

        arm_position = arm_pos.read()
        if arm_position > max_opening_pos:
            arm_pos_max += 1
    
        current_avg = current_average()
        
        print(current_avg, ", ", arm_position)

    hold()
    current_sense.disable_reporting() # Start reporting from current_sense
    arm_pos.disable_reporting()

def retract(cylinder_size):
    print("Closing gripper...")
    current_sense.enable_reporting() # Start reporting from current_sense
    arm_pos.enable_reporting()
    CCW(20)
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
    hold()
    current_sense.disable_reporting() # Start reporting from current_sense
    arm_pos.disable_reporting()

# try:
#     while True:
#         arm_pos.enable_reporting()
#         print(arm_pos.read())
#         time.sleep(0.1)
#         arm_pos.disable_reporting()


# except:
#     hold()
#     print("An error occured, stopping motor")

    #extend()
    #time.sleep(2)
    #retract()
    #time.sleep(4)
   
   
   # CW(20)
   # for t in range(0, 100, 1):
   #     current_avg = current_average()
   #     print(current_avg)
   #     with open('src/p26_gripper/docs/low_pass_filter.txt', 'a') as outfile:
   #         #writer = csv.writer(outfile)
   #         #writer.write(current_avg)
   #         outfile.write(str(current_avg))
   #         outfile.write("\n")
   #     time.sleep(0.1)
   #     t += 1 

   # hold()




    #CW(5)
    #for freq in np.array([100, 500, 1000, 2000, 4000, 5000, 7000, 10000, 12000, 15000, 20000]):
    #    motor.ChangeFrequency(freq)
    #    print(freq)
    #    time.sleep(5)

