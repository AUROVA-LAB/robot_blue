#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "../include/kalman_filter.h"
#include <tf/tf.h>
#include <math.h>

KalmanFilterPtr estimation_rpy = new KalmanFilter();

sensor_msgs::Imu virtual_imu_msg;
sensor_msgs::Imu originl_imu_msg;

static int first_vel = 0;
static int first_run = 1;
static float roll_offset = 0.0;
static float pitch_offset = 0.0;
static float yaw_offset = 0.0;

void rpyFromGpsVelocity(float& roll, float& pitch, float& yaw, float x, float y, float z)
{
  //extract yaw from velocity
  float vel_xy = sqrt(pow(x, 2) + pow(y, 2));
  if (vel_xy >= MIN_SPEED){
    yaw = acos(x / vel_xy);
    if (y < 0.0){
      yaw = -1 * yaw;
    }
    first_vel = 1;
  }else{
    yaw = MIN_CODE;
  }

}

void imuCallback(const sensor_msgs::Imu& Imu_msg)
{
  originl_imu_msg.angular_velocity.x = Imu_msg.angular_velocity.x;
  originl_imu_msg.angular_velocity.y = Imu_msg.angular_velocity.y;
  originl_imu_msg.angular_velocity.z = Imu_msg.angular_velocity.z;
}

void gpsCallback(const geometry_msgs::TwistWithCovarianceStamped& gps_msg)
{
  float roll, pitch, yaw;
  rpyFromGpsVelocity(roll, pitch, yaw, gps_msg.twist.twist.linear.x, gps_msg.twist.twist.linear.y,
                     gps_msg.twist.twist.linear.z);

  if (first_run && first_vel)
  {
    ROS_INFO("first yaw: [%f]", yaw);
    yaw_offset = yaw - estimation_rpy->X_[2][0];
    first_run = 0;
  }

  yaw = yaw - yaw_offset;
  estimation_rpy->correct(0.0, 0.0, yaw);

  //debug
  //ROS_INFO("yaw: [%f]", yaw);
}

int createVirtualImu(void)
{

  ////SUBSCRIBER
  ros::NodeHandle nh;
  ros::Subscriber original_imu;
  ros::Subscriber gps_velocity;
  original_imu = nh.subscribe("imu/data", 0, imuCallback);
  gps_velocity = nh.subscribe("rover/fix_velocity", 0, gpsCallback);

  ////PUBLISHER
  ros::Publisher imu_publisher = nh.advertise < sensor_msgs::Imu > ("virtual_imu_data", 0);

  ros::Rate loop_rate(20);
  double t_1 = 0.0;
  double t_2 = (double)ros::Time::now().toSec();
  while (ros::ok())
  {

    //calculate delta time
    t_1 = (double)ros::Time::now().toSec();
    float delta_t = (float)(t_1 - t_2);
    t_2 = (double)ros::Time::now().toSec();

    //orientation calculations
    estimation_rpy->predict(delta_t, originl_imu_msg.angular_velocity.x, originl_imu_msg.angular_velocity.y,
                            originl_imu_msg.angular_velocity.z);
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(estimation_rpy->X_[0][0], estimation_rpy->X_[1][0],
                                                            estimation_rpy->X_[2][0]);

    //create message
    virtual_imu_msg.header.stamp = ros::Time::now();
    virtual_imu_msg.header.frame_id = "imu_link";
    virtual_imu_msg.orientation.x = quaternion[0];
    virtual_imu_msg.orientation.y = quaternion[1];
    virtual_imu_msg.orientation.z = quaternion[2];
    virtual_imu_msg.orientation.w = quaternion[3];
    virtual_imu_msg.orientation_covariance[0] = originl_imu_msg.orientation_covariance[0];
    virtual_imu_msg.orientation_covariance[4] = originl_imu_msg.orientation_covariance[4];
    virtual_imu_msg.orientation_covariance[8] = originl_imu_msg.orientation_covariance[8];

    //publish topic
    imu_publisher.publish(virtual_imu_msg);

    ros::spinOnce();
    loop_rate.sleep();

    //debug
    //ROS_INFO("yaw: [%f]", estimation_rpy->X_[2][0]);
  }

  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "virtual_imu");

  createVirtualImu();

  return 0;
}
