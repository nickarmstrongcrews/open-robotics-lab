Copyright 2011, 2012 Massachusetts Institute of Technology

This work is sponsored by the Department of the Air Force under Air Force Contract #FA8721-05-C-0002.
Opinions, interpretations, conclusions and recommendations are those of the authors and are not necessarily endorsed by the United States Government.


Motivation
----------
The Kinect's pose on the robot needs to be specified with pretty darn good accuracy for sensitive applications, like mapping, to work properly.

Due to slight differences in Kinect manufacturing and robot construction, the default supplied parameters won't be quite right.

Purpose
-------
This package is intended to determine the *pitch* correction needed; roll and yaw are pretty good as they come, and mapping is less sensitive to slight errors therein. A linear offset (translation) is even less important, so we also ignore that here.

Steps
-----
0) The below assumes you have the modified files we've already supplied... if not, you can find them in ./sample_files_to_modify/ (so if you can't find what to change in the instructions below, look at the files in this folder and it should be obvious)

1) Place robot on the floor, with a big open space in front of it. Run the following launch script, play with the pitch correction until the floor in the grid lines up with the floor clearly visible in the Kinect output; in the terminal window, view correction in radians and record for next step.

roslaunch launch/kinect_calibration.launch

2) Edit the file below, putting in the correction in radians from above:

roscd turtlebot_bringup
sudo gedit kinect.launch
# change line  <node pkg="tf" ... args="0 0 0 0 [pitchCorrection] 0 /kinect_depth_frame ... " />

3) Also edit the following file, this time entering the *negative* of the correction in radians from above.

roscd turtlebot_description/urdf
sudo gedit turtlebot_calibration.xacro
# change line: <property name="turtlebot_calib_cam_rp" value="-[pitchCorrection]" />

4) Reboot robot (sorry, the "right" way of turtlebot-stop/start doesn't work for me...)

5) Launch kinect_calibration once more, and make sure it looks good (with zero additional correction).
