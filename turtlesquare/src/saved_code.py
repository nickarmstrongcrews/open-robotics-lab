
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
			# note the change from "==" to ">", this will publish for all profiles as long as the shape deadman (button B) is pushed
			if self.run_ok > 0:
				cmd_vel_msg = Twist()
				cmd_vel_msg.linear.x = lin_cmd		
				cmd_vel_msg.angular.z = ang_cmd		

				# Keep this in run_ok to allow joy_callback to be a joystick (otherwise zeros would be written)
				self.cmd_vel_pub.publish(cmd_vel_msg)

