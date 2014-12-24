LEGO Mindstorms Testing Platform
===============================

A testing platform for Differential drive robots that is built upon the LEGO Mindstorms kits.
The software is using the open source ROS framework. The roscore runs on the Central PC and 
all the robots have nodes running on them which allows for coordination among the robots. 
An aerial camera is connected to the central PC which locates and tracks all the robots using
color markers. Each robot uses odometry along with the camera estimate to get an accurate
localization data.

EV3dev
======
This project provides a Debian based Ubuntu OS for the EV3 hardware. You can find detailed
documentation and resources in their [project page](https://github.com/ev3dev/ev3dev).
