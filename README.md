# Kinect_scan
A program for scanning a room with kinect to avoid fourniture while in a VR environment 


This contains a breif guide how to install / run the code.

## Installation instructions
The tools require full ROS installation. The installation assumes you have Ubuntu 16.04 LTS [ROS Kinetic].

## Install ROS:
Please refer to http://wiki.ros.org/kinetic/Installation

Download the source tree into your catkin workspace (here we assume ~/catkin_ws):

`$ cd ~/catkin_ws/src`

`$ git clone https://github.com/hpaugam33/kinect_scan.git

## Compile the source

`$ cd ~/catkin_ws`

`$ catkin_make --pkg kinect_scan

## Then run:

$ roslaunch kinect_scan kinect_scan.launch 

![alt text](https://github.com/hpaugam33/kinect_scan/blob/master/docs/classroom.png)

![alt text](https://github.com/hpaugam33/kinect_scan/blob/master/docs/segmentation.png)




