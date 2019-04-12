#!/usr/bin/env python 

import rospy, actionlib
import thread 

from control_msgs.msg import SingleJointPositionAction
from control_msgs.msg import GripperCommandAction
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64
from multiprocessing import Lock
from math import asin, sin
from arbotix_python.parallel_convert import *

class JointController(object):
	def __init__(self):
	
		#self.joints=rospy.get_param('~joints')
		#self.a_s_p =self.joints['arm_shoulder_pan_joint']
		
		#self.max_angle =self.a_s_p['max_angle']
		self.max_angle=3.14
	#	self.max_speed =self.a_s_p['max_speed']
		self.min_angle = -3.14
	#	self.min_angle =self.a_s_p['min_angle']

	def setCommand(self, goal):
		if goal.position < self.min_angle or goal.position > self.max_angle:
			rospy.logerr("command exceeds the alowed angle")
			return false 
		position = goal.position
		self.pub.publish(position)
		rospy.loginfo("arm_shoulder_pan publish servo goal : %.4f", position)
		return True	

#arm_shoulder_pan_joint service
class ArmShoulderPanController(JointController):

	def __init__(self):
		self.max_angle=1.60
		self.min_angle=-1.60
		self.pub = rospy.Publisher('arm_shoulder_pan_joint/command',Float64, queue_size=5)
		self.Motor_ArmShoulderPan()
   
	def Motor_ArmShoulderPan(self):
		
		self.server = actionlib.SimpleActionServer('arm_shoulder_pan_action', SingleJointPositionAction, execute_cb = self.arm_shoulder_pan_action, auto_start = False)
		self.server.start()
		rospy.loginfo("arm_shoulder_pan server starts")
		rospy.spin

	def arm_shoulder_pan_action(self, goal):
		super(ArmShoulderPanController,self).setCommand(goal)
		rospy.loginfo("Command set")
		if self.server.is_preempt_requested():
			self.server.set_preempted()
			rospy.logwarn('arm_shoulder_pan: Preempted.')
			return
		self.server.set_succeeded()
		rospy.loginfo('arm_shoulder_pan: Succeeded!')

class ArmForearmJointController(JointController):
	def __init__(self):
		self.max_angle=3.14
		self.min_angle=-3.14
		self.pub = rospy.Publisher('arm_forearm_joint/command',Float64, queue_size=5)
		self.Motor_ArmForearmJoint()
   
	def Motor_ArmForearmJoint(self):
		
		self.server = actionlib.SimpleActionServer('arm_forearm_joint_action', SingleJointPositionAction, execute_cb = self.arm_forearm_joint_action, auto_start = False)
		self.server.start()
		rospy.loginfo("arm_forearm_joint server starts")
		rospy.spin

	def arm_forearm_joint_action(self, goal):
		super(ArmForearmJointController,self).setCommand(goal)
		if self.server.is_preempt_requested():
			self.server.set_preempted()
			rospy.logwarn('arm_forearm_joint: Preempted.')
			return
		self.server.set_succeeded()
		rospy.loginfo('arm_forearm_joint: Succeeded!')

class ArmBicepJointController(JointController):
	def __init__(self):
		self.max_angle=3.14
		self.min_angle=-3.14
		self.pub = rospy.Publisher('arm_bicep_joint/command',Float64, queue_size=5)
		self.Motor_ArmBicepJoint()
   
	def Motor_ArmBicepJoint(self):
		
		self.server = actionlib.SimpleActionServer('arm_bicep_joint_action', SingleJointPositionAction, execute_cb = self.arm_bicep_joint_action, auto_start = False)
		self.server.start()
		rospy.loginfo("arm_bicep_joint server starts")
		rospy.spin

	def arm_bicep_joint_action(self, goal):
		super(ArmBicepJointController,self).setCommand(goal)
		if self.server.is_preempt_requested():
			self.server.set_preempted()
			rospy.logwarn('arm_bicep_joint: Preempted.')
			return
		self.server.set_succeeded()
		rospy.loginfo('arm_bicep_joint: Succeeded!')

class ArmWristFlexJointController(JointController):
	def __init__(self):
		self.max_angle=3.14
		self.min_angle=-3.14
		self.pub = rospy.Publisher('arm_wrist_flex_joint/command',Float64, queue_size=5)
		self.Motor_ArmWristFlexJoint()
   
	def Motor_ArmWristFlexJoint(self):
		
		self.server = actionlib.SimpleActionServer('arm_wrist_flex_joint_action', SingleJointPositionAction, execute_cb = self.arm_wrist_flex_joint_action, auto_start = False)
		self.server.start()
		rospy.loginfo("arm_wrist_flex_joint server starts")
		rospy.spin

	def arm_wrist_flex_joint_action(self, goal):
		super(ArmWristFlexJointController,self).setCommand(goal)
		if self.server.is_preempt_requested():
			self.server.set_preempted()
			rospy.logwarn('arm_wrist_flex_joint: Preempted.')
			return
		self.server.set_succeeded()
		rospy.loginfo('arm_wrist_flex_joint: Succeeded!')

class GripperController:
	
	def __init__(self):
		self.tolerance = 0.001
		self.pad_width = 0.002
		self.min_opening = 0.002 
		self.max_opening = 0.031
		self.center = 0.0
		self.finger_length = 0.02
		self.invert = False
		self.joint = 'gripper_joint'
		self.model = 'parallel'
		self.pub = rospy.Publisher('gripper_joint/command',Float64, queue_size = 5)
		rospy.loginfo("started %s Gripper Controller", self.model)
		self.convertor = ParallelConvert(self.joint)

		self.server = actionlib.SimpleActionServer('gripper_action', GripperCommandAction, execute_cb = self.actionCb, auto_start = False)
		self.server.start()
		rospy.loginfo("Gripper server starts")
		rospy.spin()
	
	
	def setCommand(self,goal):
		if goal.position > self.max_opening or goal.position < self.min_opening:
			rospy.logerr("Command (%f) exceeds opening limits (%f, %f)", goal.position, self.max_opening, self.min_opening)
			return False
		angle = self.convertor.widthToAngle(goal.position)
		
		if self.invert:
			res = -angle + self.center
		else:
			res = angle + self.center 
		self.pub.publish(res)
		rospy.loginfo("Gripper publish servo goal: %.4f rad", res)
		return True
	def actionCb(self, goal):
		rospy.loginfo('Gripper controller action goal recieved: %f m' % goal.command.position)
		if self.setCommand(goal.command) == False:
			self.server.set_aborted(text= "Command exceeds range")
			return
				
		if self.server.is_preempt_requested():
			self.server.set_preempted()
			rospy.logwarn('arm_wrist_flex_joint: Preempted.')
			return
		self.server.set_succeeded()
		rospy.loginfo('arm_wrist_flex_joint: Succeeded!')

		


if __name__=='__main__':
	try:
		rospy.init_node('joints_control')
		ArmShoulderPanController()
		ArmForearmJointController()
		ArmBicepJointController()
		ArmWristFlexJointController()
		GripperController()
	except rospy.ROSInterruptException:
		rospy.loginfo('Failure')	
