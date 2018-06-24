#!/usr/bin/env python

import sys
import roslib
import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Int64
from tf.transformations import quaternion_about_axis


def imu_publisher():
    vel_x = 0.0
    vel_y = 0.0
    vel_theta = 0

    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)

    rospy.init_node("butiaros_imu_publisher",anonymous=True)
    rospy.loginfo ("Topic = /imu_data")

    x = 0.0
    y = 0.0
    theta = 0.0

    current_time = rospy.get_rostime() #en segundos
    last_time = current_time

    rate = rospy.Rate(20) #1 hz (1 seg)
    i = 0
    while not rospy.is_shutdown():
        #rospy.loginfo ("Making Odometry message...")
        #ROS: Imu
        seq = i
        imu_msg = Imu() 

        imu_msg.header.seq = seq
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "imu_link"

        # new
        imu_msg.orientation.x = 1.0;
        imu_msg.orientation.y = 1.0;
        imu_msg.orientation.z = 1.0;
        imu_msg.orientation.w = 1.0;  


        # imu_msg.orientation e imu_msg.orientation_covariance
        imu_msg.orientation_covariance[0] = -1

        # imu_msg.linear_acceleration (m/s2)
        imu_msg.linear_acceleration.x = 0.0 
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        p_cov = np.array([0.0]*9).reshape(3,3)
        p_cov[0,0] = 0.001
        p_cov[1,1] = 0.001
        p_cov[2,2] = 0.001
        imu_msg.linear_acceleration_covariance = tuple(p_cov.ravel().tolist())

        # imu_msg.angular_velocity (rad/sec)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        p_cov2 = np.array([0.0]*9).reshape(3,3)
        p_cov2[0,0] = 0.001
        p_cov2[1,1] = 0.001
        p_cov2[2,2] = 0.001
        imu_msg.angular_velocity_covariance = tuple(p_cov2.ravel().tolist())

        #rospy.loginfo ("Sending Odometry message...")
        imu_pub.publish(imu_msg)

        i = i + 1
        last_time = current_time
        current_time = rospy.get_rostime() # in seconds
        rate.sleep()
        #end_while
    #end_def

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
