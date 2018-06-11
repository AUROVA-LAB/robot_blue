
#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
//#include <tf/tf.h>
#include <time.h>

double covariance;
ackermann_msgs::AckermannDriveStamped estimated_ackermann_state;

void ackermann_callback(const ackermann_msgs::AckermannDriveStamped& estimated_ackermann_state_msg)
{
	estimated_ackermann_state.drive.speed          = estimated_ackermann_state_msg.drive.speed;
	estimated_ackermann_state.drive.steering_angle = estimated_ackermann_state_msg.drive.steering_angle;
}

void covariance_callback(const ackermann_msgs::AckermannDriveStamped& covariance_ackermann_state_msg)
{
	covariance = covariance_ackermann_state_msg.drive.speed;
}

int main (int argc, char** argv) {

  const float WHEELBASE_METERS = 1.05;
	const int   ROWS             = 6;
	const int   COLUMNS          = 6;

	int i, j;
	float orientation_z_prev = 0;
	float pose_x_prev        = 0;
	float pose_y_prev        = 0;
	double t_1;
	double t_2;



  ros::init (argc, argv, "ackermann_to_odom");
  ros::NodeHandle nh;

	geometry_msgs::TransformStamped odom_trans;
	static tf::TransformBroadcaster broadcaster;

  //// PUBLISHER
  nav_msgs::Odometry odometry;
  ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>("odometry", 0);
  //ros::Publisher odometry_publisher("odometry", &odometry);
  //nh.advertise(odometry_publisher);

  //// SUBSCRIBERS
  ros::Subscriber estimated_ackermann_subscriber;
  ros::Subscriber covariance_ackermann_subscriber;
  estimated_ackermann_subscriber = nh.subscribe ("estimated_ackermann_state", 0, ackermann_callback);
  covariance_ackermann_subscriber = nh.subscribe ("covariance_ackermann_state", 0, covariance_callback);
  //ros::Subscriber<ackermann_msgs::AckermannDriveStamped> ackermann_subscriber("estimated_ackermann_state",
  //                                                                            &ackermann_callback);
  //nh.subscribe(ackermann_subscriber);

  ros::Rate loop_rate(10);
  t_2 = (double)ros::Time::now().toSec();
  while (ros::ok()){

	  if (covariance == 0.0) covariance = 0.01;

		/////////////////////////////////////////////////
		/////// Calculate odometry_publisher ////////////
		t_1 = (double)ros::Time::now().toSec();
		float delta_t = (float)(t_1 - t_2);
		t_2 = (double)ros::Time::now().toSec();
    float lineal_speed    = estimated_ackermann_state.drive.speed;

		// Angle
    double steering_radians = estimated_ackermann_state.drive.steering_angle * M_PI / 180.0;
		float angular_speed_z   = (lineal_speed / WHEELBASE_METERS) * tan(steering_radians);
		float orientation_z     = orientation_z_prev + angular_speed_z*delta_t;
    if (abs(orientation_z) >= (2.0 * M_PI))
		    orientation_z = orientation_z - (2.0 * M_PI);
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, orientation_z);

		// pose
		float lineal_speed_x = lineal_speed * cos(orientation_z);
	  float lineal_speed_y = lineal_speed * sin(orientation_z);
		float pose_x         = pose_x_prev + lineal_speed_x * delta_t;
		float pose_y         = pose_y_prev + lineal_speed_y * delta_t;

    // For next step
		orientation_z_prev = orientation_z;
		pose_x_prev        = pose_x;
		pose_y_prev        = pose_y;
		/////////////////////////////////////////////////
		/////////////////////////////////////////////////

		// Header
	  odometry.header.stamp          = ros::Time::now();
	  odometry.header.frame_id       = "odom";
	  odometry.child_frame_id        = "base_link";

		// Twist
	  odometry.twist.twist.linear.x  = lineal_speed_x;
		odometry.twist.twist.linear.y  = lineal_speed_y;
		odometry.twist.twist.linear.z  = 0;
		odometry.twist.twist.angular.x = 0;
		odometry.twist.twist.angular.y = 0;
	  odometry.twist.twist.angular.z = angular_speed_z;
		for (i=0; i<COLUMNS; i++){
			odometry.twist.covariance[i*ROWS+i] = covariance;
		}

		// Pose
		odometry.pose.pose.position.x    = pose_x;
		odometry.pose.pose.position.y    = pose_y;
		odometry.pose.pose.position.z    = 0;
		odometry.pose.pose.orientation.x = quaternion[0];
		odometry.pose.pose.orientation.y = quaternion[1];
		odometry.pose.pose.orientation.z = quaternion[2];
		odometry.pose.pose.orientation.w = quaternion[3];
		for (i=0; i<COLUMNS; i++){
			odometry.pose.covariance[i*ROWS+i] = covariance;
		}

    // tf mensaje
		odom_trans.header.frame_id         = "odom";
    odom_trans.child_frame_id          = "base_link";
		odom_trans.header.stamp            = ros::Time::now();
    odom_trans.transform.translation.x = pose_x;
    odom_trans.transform.translation.y = pose_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation      = tf::createQuaternionMsgFromYaw(orientation_z);

		// Topic publisher
	  odometry_publisher.publish(odometry);
		//broadcaster.sendTransform(odom_trans);
	  //odometry_publisher.publish(&odometry);

	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}
