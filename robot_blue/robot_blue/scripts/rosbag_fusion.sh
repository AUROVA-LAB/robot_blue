#!/bin/bash
rosbag record /tf_static /camera/color/image_raw /camera/color/camera_info /velodyne_packets /estimated_ackermann_state /covariance_ackermann_state /imu/data
