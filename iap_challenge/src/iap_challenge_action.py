#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from iap_challenge.cfg import IapChallengeActionConfig
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

class IapChallengeAction:

    # node initialization
    def __init__(self):
        self.goal_z = rospy.get_param('goal_z', 0.6)
        self.x_scale = rospy.get_param('x_scale', 7.0)
        self.z_scale = rospy.get_param('z_scale', 2.0)

        # dynamic_reconfigure server
        self.srv = Server(IapChallengeActionConfig, self.dynamic_reconfigure_callback)

        # subscribe to marker topic
        rospy.Subscriber("~marker", Marker, self.marker_callback)

        # we'll publish on ~cmd_vel
        self.cmd_vel_pub = rospy.Publisher("~cmd_vel", Twist)


    # callback for marker (ball location) messages
    def marker_callback(self, marker_msg):
        cmd_vel_msg = Twist()

        # marker is valid if marker_msg.pose.position.z is not zero
        if marker_msg.pose.position.z != 0.0:
            cmd_vel_msg.linear.x = (marker_msg.pose.position.z - self.goal_z) * self.z_scale
            cmd_vel_msg.angular.z = -marker_msg.pose.position.x * self.x_scale
        else:
            # marker not valid, ball not found by perception
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.angular.z = 0

        # publish velocity command
        self.cmd_vel_pub.publish(cmd_vel_msg)

    # callback for the dynamic reconfigure server
    def dynamic_reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfiugre Request: {goal_z}, {x_scale},\ 
          {z_scale}""".format(**config))
        self.goal_z = config.goal_z
        self.x_scale = config.x_scale
        self.z_scale = config.z_scale
        return config

if __name__ == "__main__":
    rospy.init_node("iap_challenge_action")
    node = IapChallengeAction()
    rospy.spin()
