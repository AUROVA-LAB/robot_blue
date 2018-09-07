#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include <time.h>

double covariance;
ackermann_msgs::AckermannDriveStamped estimated_ackermann_state;
sensor_msgs::Imu virtual_imu_msg;

void imu_callback(const sensor_msgs::Imu& Imu_msg)
{
  virtual_imu_msg.orientation.x = Imu_msg.orientation.x;
  virtual_imu_msg.orientation.y = Imu_msg.orientation.y;
  virtual_imu_msg.orientation.z = Imu_msg.orientation.z;
  virtual_imu_msg.orientation.w = Imu_msg.orientation.w;
}

void ackermann_callback(const ackermann_msgs::AckermannDriveStamped& estimated_ackermann_state_msg)
{
  estimated_ackermann_state.drive.speed = estimated_ackermann_state_msg.drive.speed;
  estimated_ackermann_state.drive.steering_angle = estimated_ackermann_state_msg.drive.steering_angle;
}

void covariance_callback(const ackermann_msgs::AckermannDriveStamped& covariance_ackermann_state_msg)
{
  covariance = covariance_ackermann_state_msg.drive.speed;
}

int main(int argc, char** argv)
{

  const float WHEELBASE_METERS = 1.05;
  const int ROWS = 6;
  const int COLUMNS = 6;

  int i, j;
  float orientation_z = 0, orientation_z_prev = 0;
  float pose_x_prev = 0;
  float pose_y_prev = 0;
  double t_1;
  double t_2;

  ros::init(argc, argv, "ackermann_to_odom");
  ros::NodeHandle nh;

  geometry_msgs::TransformStamped odom_trans;
  geometry_msgs::TransformStamped base_trans;
  geometry_msgs::TransformStamped base_links;
  tf::TransformBroadcaster broadcaster;

  //// PUBLISHER
  nav_msgs::Odometry odometry;
  ros::Publisher odometry_publisher = nh.advertise < nav_msgs::Odometry > ("odometry", 0);

  //// SUBSCRIBERS
  ros::Subscriber estimated_ackermann_subscriber;
  ros::Subscriber covariance_ackermann_subscriber;
  ros::Subscriber virtual_imu;
  estimated_ackermann_subscriber = nh.subscribe("estimated_ackermann_state", 0, ackermann_callback);
  covariance_ackermann_subscriber = nh.subscribe("covariance_ackermann_state", 0, covariance_callback);
  virtual_imu = nh.subscribe("virtual_imu_data", 0, imu_callback);

  ros::Rate loop_rate(10);
  t_2 = (double)ros::Time::now().toSec();
  while (ros::ok())
  {

    if (covariance == 0.0)
      covariance = 0.01;

    /////////////////////////////////////////////////
    /////// Calculate odometry_publisher ////////////
    t_1 = (double)ros::Time::now().toSec();
    float delta_t = (float)(t_1 - t_2);
    t_2 = (double)ros::Time::now().toSec();
    float lineal_speed = estimated_ackermann_state.drive.speed;

    // Angle
    tf::Quaternion q(virtual_imu_msg.orientation.x, virtual_imu_msg.orientation.y, virtual_imu_msg.orientation.z,
                     virtual_imu_msg.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    orientation_z = yaw;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, orientation_z);
    /*
     double steering_radians = estimated_ackermann_state.drive.steering_angle * M_PI / 180.0;
     float angular_speed_z   = (lineal_speed / WHEELBASE_METERS) * tan(steering_radians);
     orientation_z           = orientation_z_prev + angular_speed_z*delta_t;
     if (abs(orientation_z) >= (2.0 * M_PI))
     orientation_z = orientation_z - (2.0 * M_PI);
     tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, orientation_z);
     */

    // pose
    ROS_INFO("yaw: [%f]", orientation_z);
    float lineal_speed_x = lineal_speed * cos(orientation_z);
    float lineal_speed_y = lineal_speed * sin(orientation_z);
    float pose_x = pose_x_prev + lineal_speed_x * delta_t;
    float pose_y = pose_y_prev + lineal_speed_y * delta_t;
    if (isnan(orientation_z))
    {
      lineal_speed_x = 0.0;
      lineal_speed_y = 0.0;
      pose_x = 0.0;
      pose_y = 0.0;
      quaternion = tf::createQuaternionFromRPY(0, 0, 0);
    }

    // For next step
    //orientation_z_prev = orientation_z;
    pose_x_prev = pose_x;
    pose_y_prev = pose_y;
    /////////////////////////////////////////////////
    /////////////////////////////////////////////////

    // Header
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";

    // Twist
    odometry.twist.twist.linear.x = lineal_speed_x;
    odometry.twist.twist.linear.y = lineal_speed_y;
    odometry.twist.twist.linear.z = 0;
    odometry.twist.twist.angular.x = 0;
    odometry.twist.twist.angular.y = 0;
    odometry.twist.twist.angular.z = 0;
    for (i = 0; i < COLUMNS; i++)
    {
      odometry.twist.covariance[i * ROWS + i] = covariance;
    }

    // Pose
    odometry.pose.pose.position.x = pose_x;
    odometry.pose.pose.position.y = pose_y;
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation.x = quaternion[0];
    odometry.pose.pose.orientation.y = quaternion[1];
    odometry.pose.pose.orientation.z = quaternion[2];
    odometry.pose.pose.orientation.w = quaternion[3];
    for (i = 0; i < COLUMNS; i++)
    {
      odometry.pose.covariance[i * ROWS + i] = covariance;
    }

    ////////////////////////////////////////////////////////////////
    // tf mensajes
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = pose_x;
    odom_trans.transform.translation.y = pose_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(orientation_z);

    base_trans.header.frame_id = "base_link";
    base_trans.child_frame_id = "velodyne";
    base_trans.header.stamp = ros::Time::now();
    base_trans.transform.translation.x = 0.55;
    base_trans.transform.translation.y = 0.0;
    base_trans.transform.translation.z = 0.0;
    base_trans.transform.rotation = tf::createQuaternionMsgFromYaw(-0.005);
    ////////////////////////////////////////////////////////////////

    // Topic publisher
    odometry_publisher.publish(odometry);
    //broadcaster.sendTransform(odom_trans);
    //broadcaster.sendTransform(base_trans);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
