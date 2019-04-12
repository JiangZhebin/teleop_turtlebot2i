#!/usr/bin/env python


"""A client to control the gripper"""

import rospy, actionlib
import thread

from control_msg.msg import GripperCommandAction
from sensor_msg.msg import JointState
from std_msgs.msg import Float64

def Gripper_client():
	client = actionlib.SimpleActionClient('gripper_action', GripperCommandAction)

	goal = control_msg.msg.GripperCommandGoal()
	goal.command.position = 0.1
		
	client.wait_for_server()
	client.send_goal(goal)
	rospy.loginfo('goal sent')


if __name__ == '__main__':
	Gripper_client()

