#!/usr/bin/python

#  Copyright 2011, 2012 Massachusetts Institute of Technology
#
#  This work is sponsored by the Department of the Air Force under Air Force Contract #FA8721-05-C-0002.
#  Opinions, interpretations, conclusions and recommendations are those of the authors and are not necessarily endorsed by the United States Government.

# Description: a simple GUI front-end to allow setting a value in ROS param space; changes are propagated in real-time.

import roslib; roslib.load_manifest('kinect_calibrate')
import rospy

import wx
import math
import threading

class TheFrame(wx.Frame):

    def __init__(self, parent, id, title):

        wx.Frame.__init__(self, parent, id, title, wx.DefaultPosition, wx.Size(300,100))
        panel = wx.Panel(self, -1)

        # grr, manual layout...
        degreeText = wx.StaticText(panel, -1, 'Kinect pitch correction (degrees):', (5, 9))
        self.sc = wx.SpinCtrl(panel, -1, '',  (230,5), (60,-1))
        self.sc.SetRange(-180,180)
        self.sc.SetValue(0)

class TheApp(wx.App):
    def OnInit(self):
        self.frame = TheFrame(None, -1, 'dynamic_param_gui.py')
        self.frame.Show(True)
        self.frame.Centre()
        return True

if __name__ == '__main__':

    rospy.init_node('dynamic_param_gui')

    app = TheApp(0)
    threading.Thread(target=app.MainLoop).start() # start GUI in a separate thread

    r = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        try:
            degs = app.frame.sc.GetValue() # grab value from GUI
            rads = degs*math.pi/180
            print "%d degrees, %f radians" %(degs, rads)
            rospy.set_param('/kinect_pitch_correction', rads) # propagate to ROS param space
        except:
            pass
