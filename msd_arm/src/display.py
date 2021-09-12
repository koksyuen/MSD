#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class TestPick():
    def __init__(self):

        # ROS initialisation
        rospy.init_node('move_group_python_display', anonymous=True)
        self.rate = rospy.Rate(1) # 1Hz

        # Moveit initialisation
        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "arduino_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        # Main Program Loop
        while not rospy.is_shutdown():
            self.display_pose()
            self.rate.sleep()

    # Display the pose of end effector to screen
    def display_pose(self):
        # obtain current pose from moveit service
        wpose = self.group.get_current_pose().pose
        print ("============ Current Pose ==============")
        quaternion = (
        wpose.orientation.x,
        wpose.orientation.y,
        wpose.orientation.z,
        wpose.orientation.w)
        # convert orientation of end effector from quaternion to euler
        euler = euler_from_quaternion(quaternion)

        print("'x':%f, 'y':%f, 'z':%f, 'a':%f, 'b':%f, 'g':%f"
                % (wpose.position.x,wpose.position.y,wpose.position.z,euler[0],euler[1],euler[2]))



if __name__=='__main__':
    TestPick()
