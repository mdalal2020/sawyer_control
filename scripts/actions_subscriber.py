#!/usr/bin/env python
import rospy
import intera_interface as ii

from sawyer_control.msg import actions


def execute_action(action_msg):

    action = action_msg.torques
    rs.enable()
    joint_names = arm.joint_names()
    joint_to_values = dict(zip(joint_names, action))
    arm.set_joint_torques(joint_to_values)

def listener():

    rospy.init_node('actions_subscriber', anonymous=True)
    rospy.Subscriber('actions_publisher', actions, execute_action)

    global arm
    global rs

    rs = ii.RobotEnable(ii.CHECK_VERSION)
    arm = ii.Limb('right')

    rospy.spin()


if __name__ == '__main__':
    listener()
