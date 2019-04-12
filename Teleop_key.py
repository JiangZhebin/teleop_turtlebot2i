#!/usr/bin/env python

from getkey import getkey, keys
import rospy
from geometry_msgs.msg import Twist

class Teleoperation_Keyboard():
	def __init__(self):
		rospy.init_node('Move', anonymous=False)
		self.Lspeed=0.1
		self.Rspeed=0.4
	def GoForward(self):
		
		
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		move_cmd = Twist()
		
		#set default speed
		move_cmd.linear.x=self.Lspeed
		move_cmd.angular.z=0
		self.cmd_vel.publish(move_cmd)
			

			
	def GoBackward(self):
		
		
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		move_cmd = Twist()

		#set default speed
		move_cmd.linear.x=-self.Lspeed
		move_cmd.angular.z=0
		
		self.cmd_vel.publish(move_cmd)
		
		
	
	def RotateLeft(self):
		
		
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		move_cmd = Twist()

		#set default speed
		move_cmd.linear.x=0
		move_cmd.angular.z=self.Rspeed
		
		self.cmd_vel.publish(move_cmd)
		

	def RotateRight(self):
		
		
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		move_cmd = Twist()

		#set default speed
		move_cmd.linear.x=0
		move_cmd.angular.z=-self.Rspeed
	
		self.cmd_vel.publish(move_cmd)
		

	def shutdown(self):
		# stop turtlebot
		rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
		self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
		rospy.sleep(1)

if __name__=="__main__":
	while(1):
		key = getkey()
		if key == keys.UP:
			Forward = Teleoperation_Keyboard()
			Forward.GoForward()
		if key == keys.DOWN:
			Forward = Teleoperation_Keyboard()
			Forward.GoBackward()
		if key == keys.LEFT:
			Forward = Teleoperation_Keyboard()
			Forward.RotateLeft()
		if key == keys.RIGHT:
			Forward = Teleoperation_Keyboard()
			Forward.RotateRight()
	






