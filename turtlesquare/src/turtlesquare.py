#!/usr/bin/env python

# load dependencies from manifest file
import roslib; roslib.load_manifest('turtlesquare')
# import main ROS python library
import rospy
import math
import numpy
from tf.transformations import euler_from_quaternion
# import the joystick and turtlesim command message datatypes
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from turtlesim.msg import Velocity
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# simple class to contain the node's variables and code
class TurtleKBNode:

	# define the initialization function (initialize global variables and sub/pub topics)
	def __init__(self):
		# variables used by any function within the class definition
		self.linear_scale = 0.2			# used to globally (and easily) scale linear command (will change shape)
		self.angular_scale = 1.7		# used to globally (and easily) scale angular command (will change shape)
		self.shape_clock = 0.0		# clock to be used by the shape function (for time based shape commands)
		self.run_ok = 1				# deadman enable / disable (see below for usage)
		self.profile = 1			# easy way to run multiple shapes from one node (see below for usage)

		self.sleep_time = 0.2			# approximately the repeat rate of the loop


		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		self.orientation_x = 0.0
		self.orientation_y = 0.0
		self.orientation_z = 0.0
		self.orientation_w = 0.0

		self.distance_threshold = 0.05
		self.angular_threshold = 3.14 / 10.0

		# log info to the screen (or to file if using a .launch file -- rather than a rosrun terminal line)		
		rospy.loginfo("In TurtleKBNode constructor.")

		# notify that you want to subscribe to the filtered odom topic AND call the a callback function when a message is received
		## this is not implemented below so it's commented out here ##
		rospy.Subscriber("/odom_combined", PoseWithCovarianceStamped, self.ekf_callback)
	
		# notify that you want to publish on cmd_vel, turtlebot_node listens to this topic so no need to remap (use the Twist data structure)
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)

		# another display to screen, these logs are useful when debugging to make sure parts of the script are executing properly
		rospy.loginfo("Done constructor.")

        def ekf_callback(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		self.z = msg.pose.pose.position.z
		rospy.loginfo("EKF Pose info [X,Y,Z]: [" + 
			      str(self.x) + " " +
			      str(self.y) + " " +   
			      str(self.z) + "]" )
		self.orientation_x = msg.pose.pose.orientation.x
		self.orientation_y = msg.pose.pose.orientation.y
		self.orientation_z = msg.pose.pose.orientation.z
		self.orientation_w = msg.pose.pose.orientation.w

	def establish_target(self, x, y, z):
		self.target_x = x
		self.target_y = y
		self.target_z = z
		rospy.loginfo("Established target = [" + str(x) + ", " + str(y) + ", " + str(z) + "]")

	def angle_to_goal(self):
		absolute_angle = math.atan2((self.y - self.target_y),
					    (self.x - self.target_x))
		current_angle = euler_from_quaternion([self.orientation_x,
						       self.orientation_y,
						       self.orientation_z,
						       self.orientation_w])
		rospy.loginfo("absolute_angle = " + str(absolute_angle))
		rospy.loginfo("current_angle = " + str(current_angle[2]))
		# Difference in angles, normalized between -pi and pi
		return numpy.unwrap([0, absolute_angle - current_angle[2]])[1]

	def distance_to_goal(self):
		d = math.sqrt( (self.x - self.target_x)**2 +
			       (self.y - self.target_y)**2 )
		rospy.loginfo("Distance to goal is " + str(d))
		return d

	def close_enough(self):
		rospy.loginfo("Are we close enough: " + str(self.distance_to_goal()) + " < " + str(self.distance_threshold) + " ?")
		return self.distance_to_goal() < self.distance_threshold

	def facing_target(self):
		return abs(self.angle_to_goal()) < self.angular_threshold

	def face_target(self):
		rospy.loginfo("In face_target()")
		while (not self.facing_target()):
			cmd_vel_msg = Twist()
			# turn toward goal angle
			cmd_vel_msg.angular.z = self.angle_to_goal()
			rospy.loginfo("Turning " + str(cmd_vel_msg.angular.z) + " radians")
			self.cmd_vel_pub.publish(cmd_vel_msg)
			rospy.sleep(self.sleep_time)     

	def drive_forward(self, distance):
		rospy.loginfo("In drive_forward(" + str(distance) + ")")
		cmd_vel_msg = Twist()
		cmd_vel_msg.linear.x = distance
		self.cmd_vel_pub.publish(cmd_vel_msg)
		rospy.sleep(self.sleep_time)    
		
	def go_to_goal(self):
		rospy.loginfo("In go_to_goal()")
		while (not self.close_enough()):
			self.face_target()
			# Drive to goal
			self.drive_forward(self.distance_to_goal())
		
	def make_square(self):
		rospy.loginfo("In make_square()")
		rospy.sleep(self.sleep_time)				
		length = 0.1		
		zero_x = self.x
		zero_y = self.y
		self.establish_target(zero_x + length, zero_y, self.z)
		self.go_to_goal()
		self.establish_target(zero_x + length, zero_y + length, self.z)
		self.go_to_goal()
		self.establish_target(zero_x, zero_y + length, self.z)
		self.go_to_goal()
		self.establish_target(zero_x, zero_y, self.z)
		self.go_to_goal()


	def simple_L(self):
		# set some values / initialize values for each loop iternation (these are local in scope, only used in this function def)
		lin_cmd = 0.0			 		# linear command, changed below
		ang_cmd = 0.0		      			# angular command, changed below

		# define profile 1 (a simple "L" shape)
		if self.run_ok == 1:
			if self.shape_clock < 1.0:
				# for the first three seconds go straight
				lin_cmd = self.linear_scale
			elif self.shape_clock < 2.4:
				# then, for the next second, rotate (zero linear)
				ang_cmd = self.angular_scale
			elif self.shape_clock < 3.0:
				# then for the next two seconds, go straight again
				lin_cmd = self.linear_scale

		# now that you set a linear and angular command you need to publish it for turtlebot_node to direct the Create
		if self.run_ok > 0:
			cmd_vel_msg = Twist()
			cmd_vel_msg.linear.x = lin_cmd		
			cmd_vel_msg.angular.z = ang_cmd		

		       	# Keep this in run_ok to allow joy_callback to be a joystick (otherwise zeros would be written)
			self.cmd_vel_pub.publish(cmd_vel_msg)


	
	# define your custom function
	def run_shape(self):
		
		# this function is set running in __main__
		# this while loop will keep the function looping until the node is stopped (e.g. Ctrl ^C)
		while not rospy.is_shutdown():
			
			# a simple way to keep a clock (by putting it at the top, here, the first time step seen below will be 0.1)
			self.shape_clock = self.shape_clock + self.sleep_time	# assumes deterministic task execution

			self.make_square()
			#self.simple_L()
			rospy.sleep(self.sleep_time)				

# end class definition (everything between class definition and here is part of the same class, by way of indentions)


# the definition of main, it says, initialize and then call run_shape
if __name__ == "__main__":
	rospy.init_node("turtlesquare")
	node = TurtleKBNode()
	#rospy.spin()
	try:
		node.run_shape()
	except rospy.ROSInterruptException: pass

