#!/bin/bash
sudo ifconfig enp2s0 192.168.1.46
sudo route add 192.168.1.201 enp2s0
rosrun rosserial_python serial_node.py /dev/arduino


