#!/usr/bin/env python

#  Copyright 2011, 2012 Massachusetts Institute of Technology
#
#  This work is sponsored by the Department of the Air Force under Air Force Contract #FA8721-05-C-0002.
#  Opinions, interpretations, conclusions and recommendations are those of the authors and are not necessarily endorsed by the United States Government.

import roslib; roslib.load_manifest('slam2D')
import rospy
import tf
import math

from geometry_msgs.msg import *

def poseDistance(p1,p2):
    deltaPositionSq = (p1.position.x - p2.position.x) ** 2 + \
                      (p1.position.y - p2.position.y) ** 2 + \
                      (p1.position.z - p2.position.z) ** 2

#    wrong way to find difference of quaternions! for now, ignoring orientation in distance computation
#    deltaOrientationSq = (p1.orientation.x - p2.orientation.x) ** 2 + \
#                      (p1.orientation.y - p2.orientation.y) ** 2 + \
#                      (p1.orientation.z - p2.orientation.z) ** 2

    deltaOrientationSq = 0 # FIXME

    return (math.sqrt(deltaPositionSq),math.sqrt(deltaOrientationSq))


if __name__ == '__main__':

    rospy.init_node('poseHistoryPublisher')

    referenceFrame = rospy.get_param('~reference_frame_id', default='/odom')
    targetFrame = rospy.get_param('~target_frame_id', default='/base_footprint')
    pubFrame = rospy.get_param('~pub_frame_id', default='/map')   # should almost always be same as referenceFrame
    minPubDist = rospy.get_param('~min_pub_distace', default=0.5) # in meters

    pub1 = rospy.Publisher('current_pose', PoseStamped)
    pub2 = rospy.Publisher('pose_history', PoseArray)

    tfl = tf.TransformListener()

    msg = PoseArray()
    msg.header.frame_id = pubFrame
    msg.poses = []

    r = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():

        try:
            t = tfl.getLatestCommonTime(referenceFrame, targetFrame)
            (trans,rot) = tfl.lookupTransform(referenceFrame, targetFrame, t)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            continue

        msg.header.stamp = t

        newPose = Pose()
        newPose.position = Point(trans[0],trans[1],trans[2])
        newPose.orientation = Quaternion(rot[0],rot[1],rot[2],rot[3])

        pub1.publish(PoseStamped(msg.header,newPose))

        # add pose and publish chain, but only if we've moved a non-trivial distance
        if len(msg.poses) > 0:
            poseDist = poseDistance(newPose, msg.poses[-1])
        if len(msg.poses) == 0 or poseDist[0] > minPubDist:
            msg.poses += [newPose]
            pub2.publish(msg)

        r.sleep()
