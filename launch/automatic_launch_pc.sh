#!/bin/bash

#start ros
#roscore &

#start ublox gps base
roslaunch ~/catkin_ws/src/ublox/ublox_gps/launch/ublox_device.launch node_name:="base" param_file_name:="blue_base"
