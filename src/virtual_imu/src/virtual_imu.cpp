
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>

sensor_msgs::Imu virtual_imu_msg;
sensor_msgs::Imu originl_imu_msg;

void imu_callback(const sensor_msgs::Imu& Imu_msg)
{
    originl_imu_msg.angular_velocity.x = Imu_msg.angular_velocity.x;
    originl_imu_msg.angular_velocity.y = Imu_msg.angular_velocity.y;
    originl_imu_msg.angular_velocity.z = Imu_msg.angular_velocity.z;
}

int create_virtual_imu (void)
{
	double t_1;
	double t_2;

	float roll,  roll_prev;
	float pitch, pitch_prev;
	float yaw,   yaw_prev;

    ros::NodeHandle nh;
    tf::Quaternion quaternion;

    //// SUBSCRIBER
    ros::Subscriber original_imu;
    original_imu = nh.subscribe ("imu/data", 0, imu_callback);

    //// PUBLISHER
    ros::Publisher imu_publisher = nh.advertise<sensor_msgs::Imu>("virtual_imu_data", 0);

    ros::Rate loop_rate(20);
    t_2 = (double)ros::Time::now().toSec();
    while (ros::ok()){

        // calculate delta time
        t_1 = (double)ros::Time::now().toSec();
        float delta_t = (float)(t_1 - t_2);
        t_2 = (double)ros::Time::now().toSec();

        // orientation calculations
        roll  = roll_prev  + delta_t*originl_imu_msg.angular_velocity.x;
        pitch = pitch_prev + delta_t*originl_imu_msg.angular_velocity.y;
        yaw   = yaw_prev   - delta_t*originl_imu_msg.angular_velocity.z;
        roll_prev  = roll;
        pitch_prev = pitch;
        yaw_prev   = yaw;
        quaternion = tf::createQuaternionFromRPY(roll, pitch, yaw);

        // create message
        virtual_imu_msg.header.stamp    = ros::Time::now();
        virtual_imu_msg.header.frame_id = "imu_link";
        virtual_imu_msg.orientation.x   = quaternion[0];
		virtual_imu_msg.orientation.y   = quaternion[1];
		virtual_imu_msg.orientation.z   = quaternion[2];
		virtual_imu_msg.orientation.w   = quaternion[3];
		virtual_imu_msg.orientation_covariance[0] = originl_imu_msg.orientation_covariance[0];
		virtual_imu_msg.orientation_covariance[4] = originl_imu_msg.orientation_covariance[4];
		virtual_imu_msg.orientation_covariance[8] = originl_imu_msg.orientation_covariance[8];

        // publish topic
		imu_publisher.publish(virtual_imu_msg);

        ros::spinOnce();
        loop_rate.sleep();

        //debug
        ROS_INFO("yaw:   [%f]", yaw);
    }

    return 0;
}


int main (int argc, char** argv)
{
	ros::init (argc, argv, "virtual_imu");

	create_virtual_imu();

    return 0;
}
