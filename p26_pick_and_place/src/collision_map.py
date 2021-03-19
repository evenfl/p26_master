#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from math import pi, atan2, cos, sin, sqrt, asin
from std_msgs.msg import String, Int64, Float32
from moveit_commander.conversions import pose_to_list
import numpy as np
#from moveit_python import PlanningSceneInterface
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPickAndPlace(object):
  """MoveGroupPickAndPlace"""
  def __init__(self):
    super(MoveGroupPickAndPlace, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_interface_tutorial',
    #                anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "p26_lefty_tcp"
    group = moveit_commander.MoveGroupCommander(group_name)

    # Misc variables
    self.resolution = 1.12
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
#    self.display_trajectory_publisher = display_trajectory_publisher
#    self.planning_frame = planning_frame
#    self.eef_link = eef_link
#    self.group_names = group_names

    # Initialize the move group for the arm
#    ur5_arm = moveit_commander.MoveGroupCommander("ur5_arm")
    # Get the name of the end-effector link
#    end_effector_link = ur5_arm.get_end_effector_link()
    # Set the reference frame for pose targets
#     reference_frame = "/base_link"
    # Set the arm reference frame accordingly
#    ur5_arm.set_pose_reference_frame(reference_frame)
    # Allow replanning to increase the odds of a solution
#    group.allow_replanning(True)
    # Allow some leeway in position (meters) and orientation (radians)
#    group.set_goal_position_tolerance(0.01)
#    group.set_goal_orientation_tolerance(0.05)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

  ## BEGIN_SUB_TUTORIAL wait_for_scene_update
  ##
  ## Ensuring Collision Updates Are Receieved
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## If the Python node dies before publishing a collision object update message, the message
  ## could get lost and the box will not appear. To ensure that the updates are
  ## made, we wait until we see the changes reflected in the
  ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
  ## For the purpose of this tutorial, we call this function after adding,
  ## removing, attaching or detaching an object in the planning scene. We then wait
  ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
    # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

    # Test if the box is in the scene.
    # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

    # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

    # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

  # If we exited the while loop without returning then we timed out
    return False
  ## END_SUB_TUTORIAL


  def add_box(self, x, y, z, i, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.group.get_planning_frame()
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z
    box_pose.pose.orientation.w = 1.0
    box_name = "box" + str(i)
    scene.add_box(box_name, box_pose, size=(self.resolution, self.resolution, self.resolution))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    #self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)




map = sensor_msgs.msg.PointCloud2()

def callback(data):
    global map
    map = data

def main():
  try:
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('collision_map', anonymous=True)

    collision_objects_publisher = rospy.Publisher('collision_object',
                                                  moveit_msgs.msg.CollisionObject,
                                                  queue_size=10)

    rospy.Subscriber("/p26_lefty/master/jetson1/kinect_decomp", sensor_msgs.msg.PointCloud2, callback)
    rospy.Subscriber("/p26_lefty/master/jetson2/kinect_decomp", sensor_msgs.msg.PointCloud2, callback)
    rospy.Subscriber("/p26_lefty/master/jetson3/kinect_decomp", sensor_msgs.msg.PointCloud2, callback)
    rospy.Subscriber("/p26_lefty/master/jetson4/kinect_decomp", sensor_msgs.msg.PointCloud2, callback)
    rospy.Subscriber("/p26_lefty/master/jetson5/kinect_decomp", sensor_msgs.msg.PointCloud2, callback)
    rospy.Subscriber("/p26_lefty/master/jetson6/kinect_decomp", sensor_msgs.msg.PointCloud2, callback)
    rospy.wait_for_message("/p26_lefty/master/jetson1/kinect_decomp", sensor_msgs.msg.PointCloud2, timeout=None)
    rospy.wait_for_message("/p26_lefty/master/jetson2/kinect_decomp", sensor_msgs.msg.PointCloud2, timeout=None)
    rospy.wait_for_message("/p26_lefty/master/jetson3/kinect_decomp", sensor_msgs.msg.PointCloud2, timeout=None)
    rospy.wait_for_message("/p26_lefty/master/jetson4/kinect_decomp", sensor_msgs.msg.PointCloud2, timeout=None)
    rospy.wait_for_message("/p26_lefty/master/jetson6/kinect_decomp", sensor_msgs.msg.PointCloud2, timeout=None)



    tutorial = MoveGroupPickAndPlace()

    tutorial.add_box(5,4,1,1)


    print( "============ Press `ENTER` to kill")
    raw_input()

    print( "============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
