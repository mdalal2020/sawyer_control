#!/usr/bin/env python
import rospy
import intera_interface as ii
from sawyer_control.srv import angle_action
from sawyer_control.srv import *

def execute_action(action_msg):
    action = action_msg.angles
    thresh =action_msg.thresh
    if thresh:
        t = 0.5
    else:
        t = 15
    joint_names = arm.joint_names()
    joint_to_values = dict(zip(joint_names, action))
    arm.move_to_joint_positions(joint_to_values, timeout = t)
    return angle_actionResponse(True)

def angle_action_server():
    rospy.init_node('angle_action_server', anonymous=True)
    global arm
    arm = ii.Limb('right')
    arm.set_joint_position_speed(0.15)
    s = rospy.Service('angle_action', angle_action, execute_action)
    rospy.spin()


if __name__ == '__main__':
    angle_action_server()

