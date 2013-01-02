#!/usr/bin/env python  

#  Copyright 2011, 2012 Massachusetts Institute of Technology
#
#  This work is sponsored by the Department of the Air Force under Air Force Contract #FA8721-05-C-0002.
#  Opinions, interpretations, conclusions and recommendations are those of the authors and are not necessarily endorsed by the United States Government.

# Description: a simple node that reads ROS parameters, representing a transform (3D rotation/translation),
#              and publishes them as a tfMessage. Works continuously, so dynamic changes to ROS params are propagated in real-time to /tf tree.

import roslib
roslib.load_manifest('kinect_calibrate')

import rospy
import tf

if __name__ == '__main__':

    rospy.init_node('param2tf')

    originalFrame = rospy.get_param('~original_frame_id', default='/base_footprint')
    correctedFrame = rospy.get_param('~corrected_frame_id', default = "%s_corrected" % originalFrame)

    tfb = tf.TransformBroadcaster()
    r = rospy.Rate(10.0) # 10 Hz
    while not rospy.is_shutdown():
        pitchCorrection = rospy.get_param('/kinect_pitch_correction',0.0)
        tfb.sendTransform( (0.0, 0.0, 0.0), # translation: (x,y,z)
                           tf.transformations.quaternion_from_euler(0.0, pitchCorrection, 0.0), # rotation: (roll,pitch,yaw)
                           rospy.Time.now(),
                           correctedFrame,
                           originalFrame)
        r.sleep()
