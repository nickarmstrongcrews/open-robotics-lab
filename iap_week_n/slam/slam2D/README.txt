1) make sure to kill any running odometry estimators (which will conflict with the one this package runs):
   rosnode kill /robot_pose_ekf
2) roslaunch planarSLAM.launch
3) your map is automatically saved continuously to /tmp/map.pgm
