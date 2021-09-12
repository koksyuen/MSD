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
                'ready_grip_spacer':{'x':0.159897, 'y':0.024380, 'z':0.009763, 'a':0.088749, 'b':1.560240, 'g':0.212793},
                'lift_up_spacer':{'x':0.158331, 'y':0.024191, 'z':0.130260, 'a':0.003280, 'b':1.557378, 'g':0.127357},
                'ready_rotate_spacer':{'x':0.095940, 'y':0.016408, 'z':0.10948, 'a':-3.141589, 'b':-0.037399, 'g':-3.017623},
                'rotate_spacer':{'x':0.095862, 'y':0.016386, 'z':0.103939, 'a':3.141578, 'b':0.038364, 'g':0.123555},
                'ready_insert_spacer':{'x':-0.004296, 'y':0.098064, 'z':0.120331, 'a':-3.141464, 'b':0.041978, 'g':1.572679},
                'insert_spacer':{'x':-0.004078, 'y':0.123051, 'z':0.130254, 'a':3.141589, 'b':0.042123, 'g':1.568014},
                'ready_push_spacer':{'x':-0.004008, 'y':0.078543, 'z':0.102594, 'a':-3.141386, 'b':0.041909, 'g':1.570595},
                'push_spacer':{'x':-0.003949, 'y':0.132791, 'z':0.129891, 'a':-3.141348, 'b':0.043083, 'g':1.572861}
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
                self.go_to_pose('ready_grip_spacer')


            # elif command.data == 2:
                self.gripper_grip(target='spacer')


            # elif command.data == 2:
                self.go_to_pose('lift_up_spacer')
                self.go_to_pose('ready_rotate_spacer')
                self.go_to_pose('rotate_spacer')
                self.go_to_pose('ready_insert_spacer')

                self.go_to_pose('insert_spacer')

            # elif command.data == 3:
                self.gripper_grip(target=None)
                self.go_to_pose('ready_push_spacer')

            # elif command.data == 4:
                self.gripper_grip(target='bolt')
                self.go_to_pose('push_spacer')

            elif command.data == 3:
                self.gripper_grip(target='spacer')


            elif command.data == 4:
                self.gripper_grip(target=None)



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
