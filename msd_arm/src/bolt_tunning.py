#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
from std_msgs.msg import String
from std_msgs.msg import *
from std_msgs.msg import UInt8
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class TestPick():
    def __init__(self):
        # a: alpha b:beta g:gamma
        self.angle_change = 0.05
        self.change = 0.005
        self.state = {
                'ready_grip_bolt':{'x':0.169556, 'y':-0.031043, 'z':-0.027287, 'a':0.104118, 'b':1.560859, 'g':-0.102543},
                'lift_up_bolt':{'x':0.168128, 'y':-0.031025, 'z':0.119080, 'a':0.189902, 'b':1.561164, 'g':-0.018369},
                'ready_rotate_bolt':{'x':0.095940, 'y':0.016408, 'z':0.10948, 'a':-3.141589, 'b':-0.037399, 'g':-3.017623},
                'rotate_bolt':{'x':0.098410, 'y':0.012887, 'z':0.113499, 'a':-3.141589, 'b':0.203590, 'g':0.085780},
                'ready_insert_bolt':{'x':0.004528, 'y':-0.097698, 'z':0.111096, 'a':-3.141462, 'b':0.086576, 'g':-1.568636},
                'insert_bolt1':{'x':0.004586, 'y':-0.112584, 'z':0.115220, 'a':3.141566, 'b':0.088181, 'g':-1.569460},
                'insert_bolt2':{'x':0.004566, 'y':-0.117518, 'z':0.117252, 'a':-3.141581, 'b':0.087580, 'g':-1.569284},
                'insert_bolt3':{'x':0.004584, 'y':-0.122518, 'z':0.117278, 'a':-3.141517, 'b':0.088337, 'g':-1.568463},
                'insert_bolt4':{'x':0.004541, 'y':-0.127546, 'z':0.118220, 'a':-3.141427, 'b':0.088800, 'g':-1.567868},
                'insert_bolt5':{'x':0.004498, 'y':-0.142640, 'z':0.118221, 'a':-3.141294, 'b':0.087371, 'g':-1.566763},
                'ready_push_bolt':{'x':0.004799, 'y':-0.092749, 'z':0.096170, 'a':-3.141476, 'b':0.085520, 'g':-1.565251},
                'push_bolt':{'x':0.004780, 'y':-0.152766, 'z':0.111154, 'a':-3.141172, 'b':0.085034, 'g':-1.563338},
                'ready_grip_bolt_head':{'x':0.004929, 'y':-0.162821, 'z':0.117054, 'a':3.141126, 'b':-0.099394, 'g':-1.562911},
                'ready_home':{'x':0.095940, 'y':0.016408, 'z':0.10948, 'a':-3.141589, 'b':-0.037399, 'g':-3.017623}
                }

        rospy.Subscriber("arm_state", UInt8, self.callback)
        rospy.Subscriber("/keyboard/keydown", std_msgs.msg.UInt16, self.keyboard_callback)

        self.gripper_state = UInt8()
        self.pub = rospy.Publisher('gripper_state', UInt8, queue_size=10)

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        self.rate = rospy.Rate(1)

        group_name = "arduino_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.group.allow_replanning(True)

        rospy.spin()


    def go_to_pose(self, goal_name):
        print("Goal: %s" %goal_name)
        goal = self.state[goal_name]
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = goal['x']
        pose_goal.position.y = goal['y']
        pose_goal.position.z = goal['z']

        quaternion = quaternion_from_euler(goal['a'], goal['b'], goal['g'])
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        self.group.set_pose_target(pose_goal)

        plan = self.group.go(wait=True)
        print ("============ Reached Goal Pose ============")
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

    def callback(self, command):

            if command.data == 1:
                self.go_to_pose('ready_grip_bolt')

            # elif command.data == 2:
                self.gripper_grip(target='bolt')

            # elif command.data == 2:
                self.go_to_pose('lift_up_bolt')
                self.go_to_pose('ready_rotate_bolt')
                self.go_to_pose('rotate_bolt')
                self.go_to_pose('ready_insert_bolt')

            # elif command.data == 3:
                self.go_to_pose('insert_bolt1')
                self.go_to_pose('insert_bolt2')
                self.go_to_pose('insert_bolt3')
                self.go_to_pose('insert_bolt4')
                self.go_to_pose('insert_bolt5')

            # elif command.data == 4:
                self.gripper_grip(target=None)
                self.go_to_pose('ready_push_bolt')
                self.gripper_grip(target='bolt')
                self.go_to_pose('push_bolt')

            # elif command.data == 5:
                self.gripper_grip(target=None)
                self.go_to_pose('ready_grip_bolt_head')
                self.gripper_grip(target='bolt_head')

            elif command.data == 2:
                self.gripper_grip(target=None)
                self.go_to_pose('ready_push_bolt')
                self.go_to_pose('ready_home')



    def keyboard_callback(self, keydown):
        key_input = keydown.data
        print("Key Down: %d" %key_input)
        wpose = self.group.get_current_pose().pose
        quaternion = (
        wpose.orientation.x,
        wpose.orientation.y,
        wpose.orientation.z,
        wpose.orientation.w)
        alpha, beta, gamma = euler_from_quaternion(quaternion)

        if (key_input == 113): #Q
            wpose.position.y += self.change
        elif (key_input == 119): #W
            wpose.position.y -= self.change
        elif (key_input == 97): #A
            wpose.position.z += self.change
        elif (key_input == 115): #S
            wpose.position.z -= self.change
        elif (key_input == 122): #Z
            beta += self.angle_change
        elif (key_input == 120): #X
            beta -= self.angle_change
        new_quaternion = quaternion_from_euler(alpha, beta, gamma)
        wpose.orientation.x = new_quaternion[0]
        wpose.orientation.y = new_quaternion[1]
        wpose.orientation.z = new_quaternion[2]
        wpose.orientation.w = new_quaternion[3]
        self.group.set_pose_target(wpose)
        plan = self.group.go(wait=True)
        print ("============ Changed Pose ============")
        print("'x':%f, 'y':%f, 'z':%f, 'a':%f, 'b':%f, 'g':%f"
                % (wpose.position.x,wpose.position.y,wpose.position.z,alpha,beta,gamma))
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

    def gripper_grip(self, target=None):
        if target==None:
            self.gripper_state.data = 0
        elif target=='spacer':
            self.gripper_state.data = 1
        elif target=='bolt':
            self.gripper_state.data = 2
        elif target=='bolt_head':
            self.gripper_state.data = 3

        self.pub.publish(self.gripper_state)

if __name__=='__main__':
    TestPick()
