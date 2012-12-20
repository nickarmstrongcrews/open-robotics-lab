#!/usr/bin/python

# load proper manifest
import roslib; roslib.load_manifest('turtle_lissa')

import rospy
import math

from sensor_msgs.msg import Joy

sim = 0

if sim:
	from turtlesim.msg import Velocity 
else:
	from geometry_msgs.msg import Twist

#from geometry_msgs.msg import PoseWithCovarianceStamped

class TurtleJoyNode:

	def __init__(self):
		self.linear_scale = 0.2
		self.angular_scale = 1.7
		self.t_start = 999.0
		self.run_ok = 0
		self.profile = 1
		
		self.last_ang_cmd = 0			# for unwraping / shortest rotatation determination

		rospy.loginfo("In TurtleJoyNode constructor.")

		rospy.Subscriber("joy", Joy, self.joy_callback)
		#rospy.Subscriber("robot_pose_ekf/odom", PoseWithCovarianceStamped, self.ekf_callback)
	
		if sim:
			self.cmd_vel_pub = rospy.Publisher("cmd_vel", Velocity)
		else:
			self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)

		rospy.loginfo("Done constructor.")

	def ekf_callback(self, ekf_msg):

		if self.run_ok == 1 & 0:
			if sim:
				cmd_vel_msg = Velocity()
			else:
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
			rospy.loginfo("Starting Lissajous:")
			self.t_start = 0
			self.profile = 3

		if joy_msg.buttons[1] & joy_msg.buttons[5]:
			# Trigger time reset, profile will begin on button up
			rospy.loginfo("Making Waves:")
			self.t_start = 0
			self.profile = 2


		if joy_msg.buttons[1] & joy_msg.buttons[4]:
			# Trigger time reset, profile will begin on button up
			rospy.loginfo("Making an L:")
			self.t_start = 0
			self.profile = 1


		if joy_msg.buttons[1]:
			self.run_ok = self.profile
		else: 
			self.run_ok = 0
			self.t_start = 999.0


		# enable joystick control if push button zero instead
		if joy_msg.buttons[0] & ~joy_msg.buttons[1]:
			
			if sim:
				cmd_vel_msg = Velocity()
				cmd_vel_msg.linear = joy_msg.axes[1]*self.linear_scale		
				cmd_vel_msg.angular = joy_msg.axes[0]*self.angular_scale		
			else:
				cmd_vel_msg = Twist()
				cmd_vel_msg.linear.x = joy_msg.axes[1]*self.linear_scale		
				cmd_vel_msg.angular.z = joy_msg.axes[0]*self.angular_scale		

		
			self.cmd_vel_pub.publish(cmd_vel_msg)
					

	def run_shape(self):

		while not rospy.is_shutdown():
			sleep_time = 0.1
			lin_cmd = 0.0
			ang_cmd = 0.0
			
			self.t_start = self.t_start + sleep_time	# assumes deteministic task execution

			if self.run_ok == 1:
				if self.t_start < 3.0:
					lin_cmd = self.linear_scale
				elif self.t_start < 4.0:
					ang_cmd = self.angular_scale
				elif self.t_start < 6.0:
					lin_cmd = self.linear_scale

			elif self.run_ok == 2:
				if self.t_start < 5.0:
					val = self.t_start
					lin_cmd = self.linear_scale
					ang_cmd = 0.9*math.sin(val*2)*self.angular_scale


			elif self.run_ok == 3:
				#rospy.loginfo("Running Shape Time: %f", self.t_start)

				time_scale = 4.5				# rate to execute (cannot exceed 0.5m/sec wheel speed) 
				lissa_dur = math.pi*2*time_scale

				xbox = 4.0/2;
				ybox = 5.0/2;
				corner = 1.0;

				if self.t_start < lissa_dur:
					a = 3.0/time_scale
					b = 4.0/time_scale

					mag_a = .4
					mag_b = .3

					offset = (math.pi/2/a)			# start near corner

					lastx = mag_a*math.sin( (self.t_start+offset-sleep_time)*a )
					xnow  = mag_a*math.sin( (self.t_start+offset)*a )
					nextx = mag_a*math.sin( (self.t_start+offset+sleep_time)*a )				
					lastdx = xnow-lastx;
					desdx  = nextx-xnow;
					#rospy.loginfo("%f %f %f :: %f %f",lastx,xnow,nextx,lastdx,desdx)


					lasty = mag_b*math.sin( (self.t_start+offset-sleep_time)*b )
					ynow  = mag_b*math.sin( (self.t_start+offset)*b )
					nexty = mag_b*math.sin( (self.t_start+offset+sleep_time)*b )				
					lastdy = ynow-lasty;
					desdy  = nexty-ynow;
					#rospy.loginfo("%f %f %f :: %f %f",lasty,ynow,nexty,lastdy,desdy)
	
					ang_cmd = math.atan2(desdy,desdx) - math.atan2(lastdy,lastdx)
					
					orig_ang = ang_cmd

					if abs(ang_cmd - self.last_ang_cmd) > math.pi:
						ang_cmd = ang_cmd-2*math.pi*self.signum(ang_cmd-self.last_ang_cmd)
						#rospy.loginfo("Unwrap: %f to %f",orig_ang*180/math.pi,ang_cmd*180/math.pi)

					self.last_ang_cmd = ang_cmd
					ang_cmd = ang_cmd / sleep_time
				
					lin_cmd = math.sqrt(desdy*desdy + desdx*desdx) / sleep_time
	
					#rospy.loginfo("%f %f",orig_ang*180/math.pi,ang_cmd*180/math.pi)
					#rospy.loginfo("Command: lin %f, ang %f at %f",lin_cmd,ang_cmd,self.t_start)
	
				elif self.t_start < lissa_dur+xbox:
					lin_cmd = 2*self.linear_scale
				elif self.t_start < lissa_dur+corner+xbox:
					ang_cmd = -self.angular_scale
				elif self.t_start < lissa_dur+ybox+corner+xbox:
					lin_cmd = 2*self.linear_scale
				elif self.t_start < lissa_dur+corner*2+ybox+xbox:
					ang_cmd = -self.angular_scale

				elif self.t_start < lissa_dur+xbox+2*corner+ybox+xbox:
					lin_cmd = 2*self.linear_scale
				elif self.t_start < lissa_dur+corner+xbox+2*corner+ybox+xbox:
					ang_cmd = -self.angular_scale
				elif self.t_start < lissa_dur+ybox+corner+xbox+2*corner+ybox+xbox:
					lin_cmd = 2*self.linear_scale
				elif self.t_start < lissa_dur+corner*2+ybox+xbox+2*corner+ybox+xbox:
					ang_cmd = -self.angular_scale


			if self.run_ok > 0:
				if sim:
					cmd_vel_msg = Velocity()
					cmd_vel_msg.linear = lin_cmd	
					cmd_vel_msg.angular = ang_cmd		
				else:
					cmd_vel_msg = Twist()
					cmd_vel_msg.linear.x = lin_cmd		
					cmd_vel_msg.angular.z = ang_cmd		

				#math.cos(x)
				#math.pi
				#math.fabs(x)
				
				# Keep this in run_ok to allow joy callback to be a joystick
				self.cmd_vel_pub.publish(cmd_vel_msg)

			
			rospy.sleep(sleep_time)				

	def signum(self, int): 
		if(int < 0): 
			return -1
		else:
			return 1			


if __name__ == "__main__":
	rospy.init_node("turtle_lissa")
	node = TurtleJoyNode()
	#rospy.spin()
	#rospy.sleep(1.0)
	try:
		node.run_shape()
	except rospy.ROSInterruptException: pass

