Command Center Multi-agent Robot Test Platform
===============================

ROS Packages used:
==================
ARUCO Augmented Reality marker detector library [link](https://github.com/pal-robotics/aruco_ros).

Bot Description:
================
LEGO Mindstorms EV3 standard differential drive chasis with an Ultrasonic sensor mounted in front.
Width: 15cm
Length: 23cm
Height: 3cm (wheel center)

Stack:
======
- Processes running on the Laptop:
-- Visual odometry using aerial camera input.
-- Map builder and server
- Processes running on each RPi:
-- ROSSerial node to proxy msgs from EV3
-- Navigation stack
-- EKF to combine odometry data.
- Processes running on each EV3:
-- base_controller
-- base_odometry
-- TF stack
-- Sensor data publisher

Laptop:
1. rosrun map_server map_server map.yaml
2. rosrun tf static_transform_publisher -0.115 -0.075 -0.03 0 0 0 base_link map 10
3. rosrun birds_eye_view mapper.py
4. 

RPi:
1. rosrun ev3_teleop listener.py

EV3:
1. ./ev3diffdrive