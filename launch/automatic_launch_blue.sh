#!/bin/bash

#start ros
#roscore &

#start arduino node
rosrun rosserial_python serial_node.py /dev/arduino &

#start ublox gps rover
roslaunch ~/iri-lab/iri_ws/src/ublox/ublox_gps/launch/ublox_device.launch node_name:="rover" param_file_name:="blue_rover" &

#start imu
#rosrun um7 um7_driver _port:=/dev/imu
