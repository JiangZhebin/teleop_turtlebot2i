#!/usr/bin/env python

"""A client to control the single joint"""

import rospy, actionlib
import thread 
import control_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from getkey import getkey, keys

def Arm_Shoulder_Pan_Client(goal):
	client = actionlib.SimpleActionClient('arm_shoulder_pan_action', control_msgs.msg.SingleJointPositionAction)
	rospy.loginfo('client created')
	client.wait_for_server()
	rospy.loginfo('waiting for server')

	client.send_goal(goal)
	rospy.loginfo('goal sent, waiting for results')
	client.wait_for_result()


def Arm_Forearm_Joint_Client(goal):
	client = actionlib.SimpleActionClient('arm_forearm_joint_action', control_msgs.msg.SingleJointPositionAction)
	client.wait_for_server()
	rospy.loginfo('waiting for server')

	client.send_goal(goal)
	rospy.loginfo('goal sent, waiting for results')
	client.wait_for_result()

def Arm_Bicep_Joint_Client(goal):
	client = actionlib.SimpleActionClient('arm_bicep_joint_action', control_msgs.msg.SingleJointPositionAction)
	client.wait_for_server()
	rospy.loginfo('waiting for server')

	client.send_goal(goal)
	rospy.loginfo('goal sent, waiting for results')
	client.wait_for_result()

def Arm_Wrist_Flex_Joint_Client(goal):
	client = actionlib.SimpleActionClient('arm_wrist_flex_joint_action', control_msgs.msg.SingleJointPositionAction)
	client.wait_for_server()
	rospy.loginfo('waiting for server')

	client.send_goal(goal)
	rospy.loginfo('goal sent, waiting for results')
	client.wait_for_result()

def Gripper_Client(goal):
	client = actionlib.SimpleActionClient('gripper_action', control_msgs.msg.GripperCommandAction)
	client.wait_for_server()
	client.send_goal(goal)
	client.wait_for_result()

if __name__=='__main__':
	
	rospy.init_node('joint_client')
		
	arm_shoulder_pan_goal = control_msgs.msg.SingleJointPositionGoal()
	arm_forearm_joint_goal = control_msgs.msg.SingleJointPositionGoal()
	arm_bicep_joint_goal = control_msgs.msg.SingleJointPositionGoal()
	arm_wrist_flex_joint_goal = control_msgs.msg.SingleJointPositionGoal()	
	gripper_goal = control_msgs.msg.GripperCommandGoal()


	arm_shoulder_pan_goal.position =0.00
	arm_forearm_joint_goal.position =0.00 
	arm_bicep_joint_goal.position = 0.00
	arm_wrist_flex_joint_goal.position =0.00 
	gripper_goal.command.position = 0.025

	#initialization 
	Arm_Shoulder_Pan_Client(arm_shoulder_pan_goal )
	Arm_Forearm_Joint_Client(arm_forearm_joint_goal)
	Arm_Bicep_Joint_Client(arm_bicep_joint_goal )
	Arm_Wrist_Flex_Joint_Client(arm_wrist_flex_joint_goal) 
	Gripper_Client(gripper_goal)



	while True:
		key = getkey()
		if key == '6':
			rospy.loginfo('enter aspg')
			arm_shoulder_pan_goal.position = arm_shoulder_pan_goal.position +0.02 
			rospy.loginfo('The present position of arm_shoulder_pan is %s', arm_shoulder_pan_goal.position)
			Arm_Shoulder_Pan_Client(arm_shoulder_pan_goal)
		if key == '4':
			arm_shoulder_pan_goal.position = arm_shoulder_pan_goal.position - 0.02
			Arm_Shoulder_Pan_Client(arm_shoulder_pan_goal)
		if key == '8':
			if arm_bicep_joint_goal.position < 1.571:
				arm_bicep_joint_goal.position = arm_bicep_joint_goal.position + 0.02
			else:
				rospy.logwarn('arm_bicep_joint reaches the maximum angle!remain unchanged')
			if arm_forearm_joint_goal.position <1.571:
				arm_forearm_joint_goal.position = arm_forearm_joint_goal.position + 0.03 
			else: 
				rospy.logwarn('arm_forearm_joint_goal reaches the maximum angle!remain unchanged')
			Arm_Bicep_Joint_Client(arm_bicep_joint_goal)
			Arm_Forearm_Joint_Client(arm_forearm_joint_goal)
		if key == '2':
			if arm_bicep_joint_goal.position > -1.571:
				arm_bicep_joint_goal.position = arm_bicep_joint_goal.position - 0.02
			else:
				rospy.logwarn('arm_bicep_joint reaches the minimum angle!remain unchanged!')
			if arm_forearm_joint_goal.position > -1.571:
				arm_forearm_joint_goal.position = arm_forearm_joint_goal.position -0.03
			else: 
				rospy.logwarn('arm_forearm_joint reahces the minimum angle! remain unchanged!')
			Arm_Bicep_Joint_Client(arm_bicep_joint_goal)
			Arm_Forearm_Joint_Client(arm_forearm_joint_goal)
		if key == '1':
			gripper_goal.command.position= 0.003
			Gripper_Client(gripper_goal)
		if key == '3':
			gripper_goal.command.position = 0.03
			Gripper_Client(gripper_goal) 	 
		if key == '0': 
			arm_shoulder_pan_goal.position =0.00
			arm_forearm_joint_goal.position =0.00 
			arm_bicep_joint_goal.position = 0.00
			arm_wrist_flex_joint_goal.position =0.00 
			gripper_goal.command.position = 0.025
	#initialization 
			Arm_Shoulder_Pan_Client(arm_shoulder_pan_goal )
			Arm_Forearm_Joint_Client(arm_forearm_joint_goal)
			Arm_Bicep_Joint_Client(arm_bicep_joint_goal )
			Arm_Wrist_Flex_Joint_Client(arm_wrist_flex_joint_goal) 
			Gripper_Client(gripper_goal)

		if key == '7':
			arm_wrist_flex_joint_goal.position = arm_wrist_flex_joint_goal.position + 0.02
			Arm_Wrist_Flex_Joint_Client(arm_wrist_flex_joint_goal)
		if key == '9':
			arm_wrist_flex_joint_goal.position = arm_wrist_flex_joint_goal.position - 0.02
			Arm_Wrist_Flex_Joint_Client(arm_wrist_flex_joint_goal)



		if key == 'k':
			arm_forearm_joint_goal.position = arm_forearm_joint_goal.position + 0.02
			Arm_Forearm_Joint_Client(arm_forearm_joint_goal)
		if key == 'l':
			arm_forearm_joint_goal.position = arm_forearm_joint_goal.position - 0.02
			Arm_Forearm_Joint_Client(arm_forearm_joint_goal)
		if key == 'n':
			arm_bicep_joint_goal.position = arm_bicep_joint_goal.position + 0.02
			Arm_Bicep_Joint_Client(arm_bicep_joint_goal)
		if key == 'm':
			arm_bicep_joint_goal.position = arm_bicep_joint_goal.position - 0.02
			Arm_Bicep_Joint_Client(arm_bicep_joint_goal)
		if key == 'h':
			arm_wrist_flex_joint_goal.position = arm_wrist_flex_joint_goal.position +0.02
			Arm_Wrist_Flex_Joint_Client(arm_wrist_flex_joint_goal)
		if key == 'j':
			arm_wrist_flex_joint_goal.position = arm_wrist_flex_joint_goal.position -0.02
			Arm_Wrist_Flex_Joint_Client(arm_wrist_flex_joint_goal)
		
			
		if key == 'q':
			break


		
