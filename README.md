A testing platform using LEGO Mindstorms kits
=============================================

A testing platform for Differential drive robots that is built upon the LEGO Mindstorms kits.
The software is using the open source ROS framework. The roscore runs on the Central PC and 
all the robots have nodes running on them which allows for coordination among the robots. 
An aerial camera is connected to the central PC which locates and tracks all the robots using
color markers. Each robot uses odometry along with the camera estimate to get an accurate
localization data.

ROS Packages used:
==================
- ROSSerial
- teleop_twist_keyboard/joystick
- [ev3-ros](https://github.com/srmanikandasriram/ev3-ros)

Bot Description:
================
Each robot is a LEGO Mindstorms EV3 standard differential drive chassis with an Ultrasonic sensor mounted in front.

* Width: 15cm
* Length: 23cm
* Height: 3cm (wheel center)

Using the testbed:
==================
On the central PC:
------------------

1. roscore
2. rosrun hawk_eye contour
3. roslaunch rosserial_server socket_node.launch
4. rosrun ev3_navigation wanderer.py => to be replaced with your algorithm of interest

5. rosrun rviz rviz => Required if you want to visualize the data

On each EV3:
------------

1. ./diffdrive_ultrasonic \<IP-address of central PC\> \<left_motor_port\> \<right_motor_port\> \<ultrasonic_sensor_port\>

Other tools:
============
- contour_standalone, present inside hawk_eye package, is useful in visually checking the various stages of processing. Helps in debugging and to ensure that each of the strips are getting detected properly.
- contour_check \<color_id\> is useful in calibrating the ranges for the different colours. The colour_ids are 1,2,3,4 corresponding to Red, Blue, Yellow and Green.
- roslaunch ev3_navigation move_base.launch => A very crude implementation which will have to be improved.
- ev3_navigation/mapper.py => an example implementation for tracking the robot movement

Extending the testbed:
=====================
- The readme file in the [ev3-ros](https://github.com/srmanikandasriram/ev3-ros) repository gives instructions on how to start writing a C program for controlling the LEGO robot with ROS. There are a number of programs but the diffdrive_ultrasonic.cpp file is the one used in the example program.
- The vision based localization is performed using the hawk_eye module. It uses standard OpenCV function calls. Possible extensions include tracking and an easier UI for calibration, arena detection and for removing the background cover.
- The ev3-ros package can be extended to implement ROS Services which can implement standard behaviours like move forward for x meters or turn by theta radians.