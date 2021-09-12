#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import UInt8, Bool
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class TestPick():
    def __init__(self):

        ##################################
        # Robotic Arm End Effector Pose  #
        ##################################

        # a: alpha b:beta g:gamma
        self.state = {
                'home':{'x':0.035037, 'y':0.004411, 'z':0.358642, 'a':0.000000, 'b':-0.000000, 'g':0.000000},
                'ready_home':{'x':0.095940, 'y':0.016408, 'z':0.10948, 'a':-3.141589, 'b':-0.037399, 'g':-3.017623},
                'ready_grip_spacer':{'x':0.159897, 'y':0.024380, 'z':0.009763, 'a':0.088749, 'b':1.560240, 'g':0.212793},
                'lift_up_spacer':{'x':0.158331, 'y':0.024191, 'z':0.130260, 'a':0.003280, 'b':1.557378, 'g':0.127357},
                'ready_rotate_spacer':{'x':0.095940, 'y':0.016408, 'z':0.10948, 'a':-3.141589, 'b':-0.037399, 'g':-3.017623},
                'rotate_spacer':{'x':0.095862, 'y':0.016386, 'z':0.103939, 'a':3.141578, 'b':0.038364, 'g':0.123555},
                'ready_insert_spacer':{'x':-0.004296, 'y':0.098064, 'z':0.120331, 'a':-3.141464, 'b':0.041978, 'g':1.572679},
                'insert_spacer':{'x':-0.004078, 'y':0.123051, 'z':0.130254, 'a':3.141589, 'b':0.042123, 'g':1.568014},
                'ready_push_spacer':{'x':-0.004008, 'y':0.078543, 'z':0.102594, 'a':-3.141386, 'b':0.041909, 'g':1.570595},
                'push_spacer':{'x':-0.003949, 'y':0.132791, 'z':0.129891, 'a':-3.141348, 'b':0.043083, 'g':1.572861},
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
                }

        #################################################################################
        # Sequence of actions to complete task                                          #
        # The sequence includes:                                                        #
        # Integer - publish the integer to arm_state topic (to Arduino)                 #
        # Key of End Effector Pose dictionary - pass the pose to moveit service         #
        # 'bolt','bolt head','spacer','release' - gripper grips the respective object   #
        #################################################################################

        # sequence of actions to insert spacer
        self.insert_spacer = ['ready_grip_spacer', 'spacer','lift_up_spacer','ready_rotate_spacer',
                            'rotate_spacer','ready_insert_spacer','insert_spacer','release',
                            'ready_push_spacer','bolt','push_spacer',2,'ready_home',
                            'release','home']


        # sequence of actions to insert bolt
        self.insert_bolt = ['ready_grip_bolt', 'bolt','lift_up_bolt','ready_rotate_bolt',
                            'rotate_bolt','ready_insert_bolt','insert_bolt1','insert_bolt2',
                            'insert_bolt3','insert_bolt4','insert_bolt5','release',
                            'ready_push_bolt','bolt','push_bolt','release','ready_grip_bolt_head',
                            'bolt_head',4
                            ]


        # sequence of actions to go back home position
        self.back_home = ['release','ready_push_bolt','ready_home','home']


        ##################################################################
        # Variable for program flow:                                      #
        # stopping - set to True when stop button is pushed              #
        # step - indicate the current action                             #
        # current_task - indicate current task (sequence of actions)     #
        ##################################################################

        self.stopping = True  # the robotic arm is stopping initially
        self.step = 0
        self.current_task = self.insert_spacer  # first task


        ###############################################################################
        # ROS initialisation                                                          #
        # Node name: move_group_python_interface                                      #
        #                                                                             #
        # Publisher:                                                                  #
        # statePub - publish current state to arm_state topic                         #
        # pub - publish object gripped by gripper to gripper_state topic              #
        #                                                                             #
        # Subscriber:                                                                 #
        # state_callback - subscribe arm_state topic that indicates current state     #
        # stop_callback - subscribe arm_stop topic that indicates stopping of system  #
        ###############################################################################

        self.gripper_state = UInt8()
        self.arm_state = UInt8()
        self.statePub = rospy.Publisher('arm_state', UInt8, queue_size=10)
        self.pub = rospy.Publisher('gripper_state', UInt8, queue_size=10)
        rospy.Subscriber("arm_state", UInt8, self.state_callback)
        rospy.Subscriber("arm_stop", Bool, self.stop_callback)
        rospy.init_node('move_group_python_interface', anonymous=True)
        self.rate = rospy.Rate(1) # 1Hz

        ##################################
        #      Moveit initialisation     #
        ##################################

        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "arduino_arm" # name of robotic arm
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.allow_replanning(True) # replan if no trajectory is found

        #################################################################################
        # Main Program Loop                                                             #
        # Loop through the current task's sequence of actions                           #
        # Sequence of actions:                                                          #
        # Integer - publish the integer to arm_state topic                              #
        # Key of End Effector Pose dictionary - pass the pose to moveit service         #
        # 'bolt','bolt head','spacer','release' - gripper grips the respective object   #
        #################################################################################

        while not rospy.is_shutdown():
            # do not do anything when stop button is pressed
            if not self.stopping:
                # loop through the current task's sequence of actions
                if self.step < len(self.current_task):
                    # Integer - publish the integer to arm_state topic
                    if isinstance(self.current_task[self.step] , int):
                        self.arm_state = self.current_task[self.step]
                        self.statePub.publish(self.arm_state)

                    # 'bolt','bolt head','spacer','release' - gripper grips the respective object
                    elif self.gripper_grip(self.current_task[self.step]) == 0:

                        # Key of End Effector Pose dictionary - pass the pose to moveit service
                        self.go_to_pose(self.current_task[self.step])
                    self.step += 1

            self.rate.sleep()

    #########################################################
    # go_to_pose                                            #
    # argument: key of End Effector Pose dictionary         #
    # send the pose of end effector to moveit service       #
    #########################################################

    def go_to_pose(self, goal_name):
        print("Goal: %s" %goal_name)
        goal = self.state[goal_name]

        pose_goal = geometry_msgs.msg.Pose()
        # coordinate of end effector
        pose_goal.position.x = goal['x']
        pose_goal.position.y = goal['y']
        pose_goal.position.z = goal['z']

        # orientation of end effector
        # convert the end effector's orientation from euler to quaternion
        quaternion = quaternion_from_euler(goal['a'], goal['b'], goal['g'])
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        # send the pose of end effector to moveit service
        self.group.set_pose_target(pose_goal)

        # plans trajectory and then moves the robotic arm
        plan = self.group.go(wait=True) # halt until robotic arm reached desired pose
        print ("==>> Reached Goal Pose\n")
        # calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # clear targets after planning with poses.
        self.group.clear_pose_targets()

    ###############################################################################
    # state_callback (subscriber call back function)                              #
    # Subcribe to arm_state topic                                                 #
    # arm_state:                                                                  #
    # 1 - insert spacer                                                           #
    # 2 - inform gantry system to start its task (by Arduino)                     #
    # 3 - insert bolt                                                             #
    # 4 - inform nut fastening system to start its task (by Arduino)              #
    # 5 - go back home position                                                   #
    ###############################################################################

    def state_callback(self, command):

            if command.data == 1:
                self.step = 0
                self.current_task = self.insert_spacer

            elif command.data == 3:
                self.step = 0
                self.current_task = self.insert_bolt

            elif command.data == 5:
                self.step=0
                self.current_task = self.back_home

    ###############################################################################
    # stop_callback (subscriber call back function)                               #
    # Stop the robotic arm when stop button is pushed                             #
    ###############################################################################

    def stop_callback(self, stopping):
        self.stopping = stopping.data
        if (self.stopping):
            # stop the robotic arm movement
            self.group.stop()
            # It is always good to clear your targets after planning with poses.
            self.group.clear_pose_targets()

    ###########################################
    # gripper_grip                            #
    # argument: object gripped by gripper     #
    # publish to gripper_state topic          #
    ###########################################

    def gripper_grip(self, target=None):
        if target=='release':
            self.gripper_state.data = 0
        elif target=='spacer':
            self.gripper_state.data = 1
        elif target=='bolt':
            self.gripper_state.data = 2
        elif target=='bolt_head':
            self.gripper_state.data = 3
        else:
            # return 0 if the object name is not included
            return 0

        print('Gripping %s' %target)
        print('==>> Finished Action\n')
        self.pub.publish(self.gripper_state)


if __name__=='__main__':
    TestPick()
