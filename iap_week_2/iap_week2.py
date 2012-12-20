#!/usr/bin/python

# load proper manifest
import roslib; roslib.load_manifest('turtle_l')

import rospy
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

class TurtleJoyNode:
	def __init__(self):
		self.linear_scale = 0.2
		self.angular_scale = 1.7
		self.t_start = 0.0
		self.run_ok = 0

		rospy.loginfo("In TurtleJoyNode constructor.")

		rospy.Subscriber("joy", Joy, self.joy_callback)
		rospy.Subscriber("robot_pose_ekf/odom", PoseWithCovarianceStamped, self.ekf_callback)
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)

	def ekf_callback(self, ekf_msg):

		if self.run_ok == 1:
			cmd_vel_msg = Twist()
			## Run profile as long as button 2 is engaged
			#if ekf_msg.header.stamp.secs < (self.t_start + 3):
			#	cmd_vel_msg.linear.x = self.linear_scale
			#elif ekf_msg.header.stamp.secs < (self.t_start + 4):
			#	cmd_vel_msg.angular.z = self.angular_scale
			#elif ekf_msg.header.stamp.secs < (self.t_start + 6):
			#	cmd_vel_msg.linear.x = self.linear_scale

			if ekf_msg.header.stamp.secs < (self.t_start + 5):
				val = (ekf_msg.header.stamp.nsecs+ekf_msg.header.stamp.secs) % (2*math.pi)
				rospy.loginfo("Rotation setting: %f2.5",val)
				
				cmd_vel_msg.linear.x = (math.cos(val)*0.5*self.linear_scale+1.5*self.linear_scale)/2
				#cmd_vel_msg.linear.x = 2*self.linear_scale
				cmd_vel_msg.angular.z = math.sin(val)*self.angular_scale

				#math.cos(x)
				#math.pi
				#math.fabs(x)
				# Keep this in run_ok to allow joy callback to be a joystick
				#self.cmd_vel_pub.publish(cmd_vel_msg)


	

	def joy_callback(self, joy_msg):
		if len(joy_msg.axes) < 2:
			rospy.logerror("bad array size")
			return

		if joy_msg.buttons[1] & joy_msg.buttons[6]:
			# Trigger time reset, profile will begin on button up
			self.t_start = joy_msg.header.stamp.secs
			#rospy.loginfo("Setting start_ok: %d",self.start_ok)

		if joy_msg.buttons[1]:
			self.run_ok = 1
		else: 
			self.run_ok = 0


		# enable joystick control if push button zero instead
		if joy_msg.buttons[0] & ~joy_msg.buttons[1]:
			cmd_vel_msg = Twist()
			cmd_vel_msg.linear.x = joy_msg.axes[1]*self.linear_scale
			cmd_vel_msg.angular.z = joy_msg.axes[0]*self.angular_scale
			self.cmd_vel_pub.publish(cmd_vel_msg)
					



if __name__ == "__main__":
	rospy.init_node("turtle_l")
	node = TurtleJoyNode()
	rospy.spin()

