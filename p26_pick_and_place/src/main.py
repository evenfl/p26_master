#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import geometry_msgs.msg

cylinder_com = geometry_msgs.msg.Point()
cylinder_dirvec = geometry_msgs.msg.Point()


def callback_com(data):
    rospy.loginfo("Received cylinder center of mass: %f , %f , %f", data.x, data.y, data.z)
#    print "Center of mass: ", round(cylinder_com.x,3), ",", round(cylinder_com.y,3), ",", round(cylinder_com.z,3)


def callback_dirvec(data):
    rospy.loginfo("Received cylinder direction vector: %f , %f , %f", data.x, data.y, data.z)
#    print "Direction vector: ", round(data.x,3), ",", round(data.y,3), ",", round(data.z,3)

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('subscriber', anonymous=True)

    rospy.Subscriber("cylinder_com", geometry_msgs.msg.Point, callback_com)
    rospy.Subscriber("cylinder_dirvec", geometry_msgs.msg.Point, callback_dirvec)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
