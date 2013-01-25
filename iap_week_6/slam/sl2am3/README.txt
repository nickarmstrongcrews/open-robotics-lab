1) roslaunch launch/sl2am3.launch
2) your 2D map is automatically saved continuously to /tmp/map.pgm
3) to save your 3D map, roslaunch launch/map_saver.launch (output file is /tmp/map3d.bt)
4) you can view your 3D map with optirun rosrun octovis octovis /tmp/map3d.bt

