#!/usr/bin/env python3.7

import rospy
from std_msgs.msg import String, Int64, Float32
import time
from actuation import extend, retract, hold

pub = rospy.Publisher('p26_lefty/gripper/position', Int64, queue_size=100)

def grip_procedure(data):
    global actuation
    actuation = data.data

rospy.init_node('gripper', anonymous=True)
rate = rospy.Rate(10)
rospy.Subscriber("p26_lefty/gripper/actuation", Int64, grip_procedure)

def open_grip():
    extend()
    gripper_pos = 1 #1 open, 2 closed
    rospy.loginfo("Gripper open")
    pub.publish(gripper_pos)

def close_grip(cylinder_size):
    retract(cylinder_size)
    gripper_pos = 2 #1 open, 2 closed
    rospy.loginfo("Gripper closed")
    pub.publish(gripper_pos)

def main():


    rospy.loginfo("Gripper ready for input...")
    hold()
    rospy.wait_for_message("p26_lefty/gripper/actuation", Int64, timeout=None)

    if actuation == 1:
        rospy.loginfo("Opening gripper...")
        open_grip()

    elif actuation == 2:
        rospy.loginfo("Closing gripper for 200mm cylinder...")
        close_grip(260)

    elif actuation == 3:
        rospy.loginfo("Closing gripper for 260mm cylinder...")
        close_grip(200)
        

    elif actuation != 1 or actuation != 2:
        rospy.loginfo("Malfunction")

try:
    while True:
        main()

except:
    hold()
    print("An error occured, stopping motor")