
#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"

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

  ros::init (argc, argv, "ackermann_to_odom");
  ros::NodeHandle nh;

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

  while (ros::ok()){

	  if (covariance == 0.0) covariance = 0.01;

	  double steering_radians        = estimated_ackermann_state.drive.steering_angle * M_PI / 180.0;
	  odometry.header.stamp          = ros::Time::now();
	  odometry.header.frame_id       = "odom";
	  odometry.child_frame_id        = "base_link";
	  odometry.twist.covariance[0]   = covariance;
	  odometry.twist.covariance[35]  = covariance;
	  odometry.twist.twist.linear.x  = estimated_ackermann_state.drive.speed;
	  odometry.twist.twist.angular.z = estimated_ackermann_state.drive.speed * tan(steering_radians) / WHEELBASE_METERS;

	  odometry_publisher.publish(odometry);
	  //odometry_publisher.publish(&odometry);

	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}
