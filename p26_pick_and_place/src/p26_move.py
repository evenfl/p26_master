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
from math import pi, atan2, cos, sin, sqrt, asin
from std_msgs.msg import String, Int64, Float32, Int8, UInt8, Bool
from moveit_commander.conversions import pose_to_list
import numpy as np
from sensor_msgs.msg import PointCloud2

#from moveit_python import PlanningSceneInterface
## END_SUB_TUTORIAL

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def normalize(vec):
    length = 0.0
    for i in range(len(vec)):
        length = length + vec[i]*vec[i]
    length = sqrt(length)
    for i in range(len(vec)):
        vec[i] = vec[i]/length
    return vec

def vec_length(vec):
    length = 0.0
    for i in range(len(vec)):
        length = length + vec[i]*vec[i]
    length = sqrt(length)
    return length

class GraspingPointCandidate:
    def __init__(self, theta, dirvec, u_init, P_com, P_eef):

        # Rodrigues' rotation formula
        w = np.cross(u_init,dirvec)

        x1 = cos(theta)
        x2 = sin(theta)

        u_rot = x1*u_init + x2*w

        u_rot = normalize(u_rot)

        self.d = vec_length([P_eef[0]-(P_com[0]+u_rot[0]), P_eef[1]-(P_com[1]+u_rot[1]), P_eef[2]-(P_com[2]+u_rot[2])])
        self.u = u_rot

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

    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)

    # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    # the robot:
    robot = moveit_commander.RobotCommander()

    # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    # to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    # to one group of joints.
    # This interface can be used to plan and execute motions on the robot:
    group_name = "p26_lefty_tcp"
    group = moveit_commander.MoveGroupCommander(group_name)#,robot_description='/p26_lefty/robot_description', ns='/p26_lefty')

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('p26_lefty/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Getting Basic Information
    # ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    # Allow replanning to increase the odds of a solution
    group.allow_replanning(True)
    # Allow some leeway in position (meters) and orientation (radians)
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.05)

  def go_to_joint_state(self):
    group = self.group

    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    joint_goal = group.get_current_joint_values()
    print "moving from "
    print joint_goal
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
    joint_goal[6] = 0
    print "moving to "
    print joint_goal

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self, x, y, z, xd, yd, zd, distance):
    group = self.group

    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    eef_pose = group.get_current_pose().pose
    P_eef = [eef_pose.position.x, eef_pose.position.y, eef_pose.position.z]
    P_com = [x,y,z]

    D = [xd, yd, zd]

    v0 = [0,0,1]
    if zd > 0.9:
        v0 = [1,0,0]

    u = np.cross(D, v0)
    u = normalize(u)

    a = np.array([])
    for i in range(360):
        a = np.append(a, GraspingPointCandidate(i*(2*np.pi)/360, D, u, P_com, P_eef))

    smallest_distance = 1000000
    for i in range(360):
        if a[i].d < smallest_distance:
            smallest_distance = a[i].d
            a_saved = a[i]

    U = a_saved.u

    W0 = [ -U[1], U[0], 0 ]
    U0 = np.cross(W0, U)
    angle_H=atan2(U[1],U[0])
    angle_P=asin(U[2])
    angle_B = atan2( np.dot(W0,D) / vec_length(W0), np.dot(U0,D) / vec_length(U0) )

    q = euler_to_quaternion( -angle_B+np.pi/4, angle_P, np.pi+angle_H )
    q = normalize(q)

    #q = euler_to_quaternion(0, np.pi/2, np.pi) # Straight down
    #q = euler_to_quaternion(0, np.pi, np.pi) # Straight backwards and upside down
    #q = euler_to_quaternion(0, 0, np.pi) # Straight forwards
    #q = euler_to_quaternion(0, 0, 0) # Straight backwards

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    pose_goal.position.x = x+distance*U[0]
    pose_goal.position.y = y+distance*U[1]
    pose_goal.position.z = z+distance*U[2]
    state = self.robot.get_current_state()
    group.set_start_state(state)
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
#    plan = move_group.plan()
#    if plan.joint_trajectory.points:  # True if trajectory contains points
#      move_success = move_group.execute(plan)
#    else:
#      rospy.logerr("Trajectory is empty. Planning was unsuccessful.")

    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()


    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    print "moving to:"
    print pose_goal
    return all_close(pose_goal, current_pose, 0.01)
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
  def add_box(self, x, y, z, l, w, h, timeout=4):
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
    box_pose.header.frame_id = self.robot.get_planning_frame()
    print(self.group.get_planning_frame())
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(l,w,h))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    #self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)



cylinder_com = geometry_msgs.msg.Point()
cylinder_dirvec = geometry_msgs.msg.Point()
cylinder = moveit_msgs.msg.CollisionObject()


def callback_com(data):
    global cylinder_com
    cylinder_com = data
    rospy.loginfo("Received cylinder center of mass: %f , %f , %f", cylinder_com.x, cylinder_com.y, cylinder_com.z)

def callback_dirvec(data):
    global cylinder_dirvec
    cylinder_dirvec = data
    rospy.loginfo("Received cylinder direction vector: %f , %f , %f", cylinder_dirvec.x, cylinder_dirvec.y, cylinder_dirvec.z)

def callback_CollisionObject(CollisionObject):
    global cylinder
    cylinder = CollisionObject

gripper_position = 0

def gripper_pos(data):
    global gripper_position
    gripper_position = data.data
    rospy.loginfo("Received gripper position: %i", gripper_position)

def wait_for_gripper():
    rospy.wait_for_message("gripper/position", UInt8, timeout=None)

def main():
  try:
    # The anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pick_and_place', anonymous=True)

    pub = rospy.Publisher('gripper/actuation', UInt8, queue_size=10)
    pub_find_cylinder = rospy.Publisher('find_cylinder', Bool, queue_size=10)
    pub_create_octomap = rospy.Publisher('create_octomap', Bool, queue_size=10)
#    rospy.Subscriber("octomap_created", String, gripper_pos)
    rospy.Subscriber("gripper/position", UInt8, gripper_pos)
    rospy.Subscriber("cylinder_com", geometry_msgs.msg.Point, callback_com)
    rospy.Subscriber("cylinder_dirvec", geometry_msgs.msg.Point, callback_dirvec)

    # Wait for the cylinder segmentation to complete.
    rospy.wait_for_message("cylinder_dirvec", geometry_msgs.msg.Point, timeout=None)
    rospy.sleep(1)
    tutorial = MoveGroupPickAndPlace()

    pub_create_octomap.publish(1)
    rospy.wait_for_message("octomap_created", Bool, timeout=None)

    while not rospy.is_shutdown():

        # Open gripper
        actuation = 1;
        pub.publish(actuation)
        #wait_for_gripper()
        while gripper_position != 1:
            break
            pub.publish(actuation)
            rospy.sleep(1)
            if gripper_position == actuation:
                break

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()

#        pub_find_cylinder.publish(1);

        # Wait for the cylinder segmentation to complete.
#        rospy.wait_for_message("cylinder_dirvec", geometry_msgs.msg.Point, timeout=None)

        #tutorial.go_to_pose_goal(cylinder_com.x+0.16, cylinder_com.y, cylinder_com.z)
        pub_create_octomap.publish(1)
        rospy.wait_for_message("octomap_created", Bool, timeout=None)
        rospy.sleep(1)

        tutorial.go_to_pose_goal(cylinder_com.x, cylinder_com.y, cylinder_com.z, cylinder_dirvec.x, cylinder_dirvec.y, cylinder_dirvec.z, 0.20)


        if gripper_position == 1:
            # Move to gripping position
            rospy.loginfo("Moving robot to target 3")
#            tutorial.go_to_pose_goal(cylinder_com.x, cylinder_com.y, cylinder_com.z, cylinder_dirvec.x, cylinder_dirvec.y, cylinder_dirvec.z, 0.14)

            #Close gripper
            actuation = 2
            pub.publish(actuation)

        else:
            # Gripper malfunction
            rospy.loginfo("Gripper malfunction")

        # Waiting for completion from gripper
        wait_for_gripper()

        if gripper_position == 2:
            rospy.loginfo("Cylinder attatched")

        tutorial.group.attach_object('cylinder')

        print "============ Press `Enter` when the cylinder is physically attatched..."
        raw_input()
        pub_create_octomap.publish(1)
        rospy.wait_for_message("octomap_created", Bool, timeout=None)
        rospy.sleep(1)

        tutorial.go_to_pose_goal(cylinder_com.x, cylinder_com.y, cylinder_com.z+0.2, cylinder_dirvec.x, cylinder_dirvec.y, cylinder_dirvec.z, 0.18)

        tutorial.go_to_pose_goal(5.42, 9.23-0.13, 0.86+0.05, 0, 0, 1, 0)

        print( "============ Press `ENTER` to release cylinder")
        raw_input()

        #Open gripper
        actuation = 1
        pub.publish(actuation)
        wait_for_gripper()

        tutorial.group.detach_object('cylinder')

        print( "============ Press `ENTER` when the cylinder is repositioned")
        raw_input()

        pub_find_cylinder.publish(1);

        # Wait for the cylinder segmentation to complete.
        rospy.wait_for_message("cylinder_dirvec", geometry_msgs.msg.Point, timeout=None)

#    tutorial.go_to_joint_state()

    print( "============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
