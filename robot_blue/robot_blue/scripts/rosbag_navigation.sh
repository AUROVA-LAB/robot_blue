#!/bin/bash
rosbag record /imu/data /estimated_ackermann_state /covariance_ackermann_state /rover/fix /rover/fix_velocity /velodyne_packets
