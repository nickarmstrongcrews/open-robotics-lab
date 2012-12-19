#!/usr/bin/env python

# load dependencies from manifest file
import roslib; roslib.load_manifest('turtlesim_gamepad_py')
# import main ROS python library
import rospy
# import the joystick and turtlesim command message datatypes
from sensor_msgs.msg import Joy
from turtlesim.msg import Velocity

# simple class to contain the node's variables and code
class TurtlesimGamepadNode:
    # class constructor; subscribe to topics and advertise intent to publish
    def __init__(self):
        self.linear_scale = 2.0
        self.angular_scale = 2.0

        # print out a message for debugging
        rospy.loginfo("In TurtlesimGamepadNode constructor.")

        # subscribe to the joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)

        # advertise that we'll publish on the turtlesim command velocity topic
        self.cmd_vel_pub = rospy.Publisher("command_velocity", Velocity)

    # the callback function for the joy topic subscription
    def joy_callback(self, joy_msg):
        # simple check for invalid joy_msg
        if len(joy_msg.axes) < 2:
            rospy.logerror("joy_msg axes array length (%d) has less than expected length (2)", len(joy_msg.axes))
            return

        # convert the joystick message to a velocity
        cmd_vel_msg = Velocity()
        cmd_vel_msg.linear = joy_msg.axes[1]*self.linear_scale
        cmd_vel_msg.angular = joy_msg.axes[0]*self.angular_scale

        # publish the velocity command message
        self.cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    rospy.init_node("turtlesim_gamepad")

    node = TurtlesimGamepadNode()

    # enter the ROS main loop
    rospy.spin()
