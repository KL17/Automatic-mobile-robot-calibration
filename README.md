# Automatic-mobile-robot-calibration

# Summary
The task of this final paper is the automatic calibration of a mobile robot, which is performed by controlling the robot using ROS (Robot Operating System - ROS), in which a node will be used to generate and send a 1x1 m square path (originally 4x4, reduced due to limited area). After that, the actual path of the robot will be recorded by OptiTrack vision system and compared to the reference path. The next step is using the initial and final location recorded by OptiTrack and calculating the calibration parameters of the robot according to the UMBMark calibration method and their implementation in order to correct errors of deviation from the reference path. At the end, the tracking of the trajectories before and after the calibration of the mobile robot is compared using a graphical display.

# System
ROS Noetic - Ubuntu 20.04 - Python 3

# Data
kvadrat2.py - script that contains frame creation functions, frame transformation functions, distance functions, speed controllers (both linear and angular), calibration and plotting function. Basically, script for launching other scripts for frame creation, using them for frame transformation between them and mobile robot, sending square trajectory (5 times clockwise and 5 times counter-clockwise) movement to a robot using that data through speed controllers, writing captured data to a .txt file, using that data for calculating calibration parameters (UMBMark calibration method of a differential robot) and plots it.

odom_pomocni.py - script for creating frame "odom_pomocni" that is used as a global coordinate frame. Square trajectory is defined by transformation between "odom_pomocni" and robot's "base_link". Also used for speed control.

odom_optitrack.py - script for creating frame "odom_optitrack" that is used as global coordinate frame. Used for transformation between "odom_optitrack" and rigid body that is defined as "robot" in OptiTrack by capturing markers that are placed on it.

fit_circle_ellipse.m - MATLAB script that finds the closest circle or ellipse for recorded data in X,Y plane. Used for finding center of robot's rotation and translating rigid body's origin to it.
