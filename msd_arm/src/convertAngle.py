#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
from msd_arm.msg import armAngle
from sensor_msgs.msg import JointState

class convertAngle():
    def __init__(self):
        # offset of robotic arm joint's angle (before publish to arduino)
        self.offset = [-1.5708, 2.3562, -3.1416, -1.5708, -3.1416]

        # custom message
        self.arm_angle = armAngle()

        # ROS initialisation
        self.pub = rospy.Publisher('joint_angle', armAngle, queue_size=10)
        rospy.init_node('angle_converter', anonymous=True)
        rospy.Subscriber("joint_states", JointState, self.callback)
        rospy.Subscriber("gripper_state", UInt8, self.gripper_callback)
        self.rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            # publish the custom message
            # custom message includes 5 joints angle and gripper state
            self.pub.publish(self.arm_angle)
            self.rate.sleep()

    # Subsribe joint_states topic (published by moveit)
    # convert the joint angle from radians to degree
    def callback(self, joint_state):
        self.arm_angle.joint0 = self.radiansToDegrees(joint_state.position[0], self.offset[0])
        self.arm_angle.joint1 = self.radiansToDegrees(joint_state.position[1], self.offset[1])
        self.arm_angle.joint2 = self.radiansToDegrees(joint_state.position[2], self.offset[2])
        self.arm_angle.joint3 = self.radiansToDegrees(joint_state.position[3], self.offset[3])
        self.arm_angle.joint4 = self.radiansToDegrees(joint_state.position[4], self.offset[4])

    #Subscribe gripper_state topic (pulished by main.py)
    def gripper_callback(self, gripper_state):
        self.arm_angle.gripper = gripper_state.data

    # convert the joint angle from radians to degree and implement offset
    def radiansToDegrees(self, position_radians, offset):
          position_radians = abs(position_radians + offset) # implement offset
          return int(position_radians * 57.2958) # convert from radians to degree

if __name__=='__main__':
    convertAngle()
