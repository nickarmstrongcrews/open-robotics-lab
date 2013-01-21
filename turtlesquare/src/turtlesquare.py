#!/usr/bin/env python

# load dependencies from manifest file
import roslib; roslib.load_manifest('turtlesquare')
# import main ROS python library
import rospy
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
            rospy.loginfo("EKF Pose info [X,Y,Z]: [" + 
                          str(msg.pose.pose.position.x) + " " +
                          str(msg.pose.pose.position.y) + " " +   
                          str(msg.pose.pose.position.z) + "]" )

	# define your custom function
	def run_shape(self):
		
		# this function is set running in __main__
		# this while loop will keep the function looping until the node is stopped (e.g. Ctrl ^C)
		while not rospy.is_shutdown():
			# set some values / initialize values for each loop iternation (these are local in scope, only used in this function def)
			sleep_time = 0.1					# approximately the repeat rate of the loop
			lin_cmd = 0.0						# linear command, changed below
			ang_cmd = 0.0						# angular command, changed below
			
			# a simple way to keep a clock (by putting it at the top, here, the first time step seen below will be 0.1)
			self.shape_clock = self.shape_clock + sleep_time	# assumes deterministic task execution

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

			# run profile #2 with an if else statement (commented out here)
			#elif self.run_ok == 2:
				# some math functions / variables that you might find helpful
				#math.cos(x)
				#math.sin(x)
				#math.pi
				#math.fabs(x)					# abs(x) will also work (not in math.)


			# now that you set a linear and angular command you need to publish it for turtlebot_node to direct the Create
			# note the change from "==" to ">", this will publish for all profiles as long as the shape deadman (button B) is pushed
			if self.run_ok > 0:
				cmd_vel_msg = Twist()
				cmd_vel_msg.linear.x = lin_cmd		
				cmd_vel_msg.angular.z = ang_cmd		

				# Keep this in run_ok to allow joy_callback to be a joystick (otherwise zeros would be written)
				self.cmd_vel_pub.publish(cmd_vel_msg)

			# still in the while loop we will sleep (aka wait), an easy way to pause
			rospy.sleep(sleep_time)				
	

	# another custom function to be used by functions above (use via self.some_other_custom_function(value)) 
	## commented out here since we have all we need ##
	#def some_other_custom_function(self, int): 


# end class definition (everything between class definition and here is part of the same class, by way of indentions)


# the definition of main, it says, initialize and then call run_shape
if __name__ == "__main__":
	rospy.init_node("turtlesquare")
	node = TurtleKBNode()
	#rospy.spin()
	try:
		node.run_shape()
	except rospy.ROSInterruptException: pass

