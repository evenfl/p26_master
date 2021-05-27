#!/usr/bin/env python3.7

import rospy
from std_msgs.msg import UInt8
import time
from actuation import extend, retract, hold

pub = rospy.Publisher('p26_lefty/gripper/position', UInt8, queue_size=100)

def grip_procedure(data): # Callback function that stores global actuation variable from topic
    global actuation
    actuation = data.data

rospy.init_node('gripper', anonymous=True)
rate = rospy.Rate(10)
rospy.Subscriber("p26_lefty/gripper/actuation", UInt8, grip_procedure)

def open_grip(): # Opening gripper and publish position
    extend()
    gripper_pos = 1 #1 open, 2 closed
    rospy.loginfo("Gripper open")
    pub.publish(gripper_pos)

def close_grip(cylinder_size): # Close gripper and publish position
    retract(cylinder_size)
    gripper_pos = 2 #1 open, 2 closed
    rospy.loginfo("Gripper closed")
    pub.publish(gripper_pos)

def main():

    rospy.wait_for_message("p26_lefty/gripper/actuation", UInt8, timeout=None)

    # Actuation value describes what action to perform, 1 open, 2 and 3 closes to corresponding cylinder
    if actuation == 1:
        rospy.loginfo("Opening gripper...")
        open_grip()

    elif actuation == 2:
        rospy.loginfo("Closing gripper for 260mm cylinder...")
        close_grip(260)

    elif actuation == 3:
        rospy.loginfo("Closing gripper for 200mm cylinder...")
        close_grip(200)
        
    else:
        rospy.loginfo("Malfunction")

try:
    gripper_pos = 0 
    pub.publish(gripper_pos) #Publishes that Raspberry initialized and ready for input
    while True:
        main()

except: # Sets all pins low if error occurs, this stops motor
    hold()
    rospy.loginfo("An error occured, stopping motor")
