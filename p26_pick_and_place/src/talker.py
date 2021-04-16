#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int64


global actuation

def gripper_pos(data):
    global gripper_position
    gripper_position = data.data


def gripper_wait():
    rospy.wait_for_message("p26_lefty/gripper", Int64, timeout=None)

    return gripper_position


def main():

    pub = rospy.Publisher('p26_lefty/gripper', Int64, queue_size=100)
    rospy.Subscriber("p26_lefty/gripper", Int64, gripper_pos)
    rospy.init_node('gripper_command', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Moving robot to target 1
    rospy.loginfo("Moving robot to target 1")
    rospy.sleep(5)

    # Open gripper
    actuation = 1;
    pub.publish(actuation)

    # Waiting for completion from gripper
    gripper_wait()

    if gripper_position == 1:
        # Move to target 3
        rospy.loginfo("Moving robot to target 3")
        rospy.sleep(5)

        #Close gripper
        actuation = 2
        pub.publish(actuation)

    else:
        # Gripper malfunction
        rospy.loginfo("Gripper malfunction")

    # Waiting for completion from gripper
    gripper_wait()

    if gripper_position == 2:
        rospy.loginfo("Finished...")

main()
