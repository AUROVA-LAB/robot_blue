#include "reactive_hokuyo_alg_node.h"
#include "reactive_hokuyo_alg.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/segmentation/sac_segmentation.h>

ReactiveHokuyoAlgNode::ReactiveHokuyoAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<ReactiveHokuyoAlgorithm>()
{
  //init class attributes if necessary
  flag_new_hokuyo_data_ = false;

  z_threshold_ = MIN_OBSTACLE_HEIGHT_;

  abs_lateral_safety_margin_ = 0.1;

  safety_width_ = VEHICLE_WIDTH_ + (2.0 * abs_lateral_safety_margin_);

  euclidean_association_threshold_ = 0.10;
  min_obstacle_radius_ = 0.03;

  closest_obstacle_point_ = OUT_OF_RANGE_;
  steering_angle_ = 0.0;

  this->loop_rate_ = 500;

  // [init publishers]
  this->front_obstacle_distance_publisher_ = this->public_node_handle_.advertise < std_msgs::Float32
      > ("front_obstacle_distance", 1);

  this->pointcloud_publisher_ = this->public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("pointcloud", 1);

  // [init subscribers]
  this->hokuyo_subscriber_ = this->public_node_handle_.subscribe("/scan", 1, &ReactiveHokuyoAlgNode::hokuyo_callback,
                                                                 this);

  this->ackermann_subscriber_ = this->public_node_handle_.subscribe("/estimated_ackermann_state", 1,
                                                                    &ReactiveHokuyoAlgNode::estimatedAckermannStateCB,
                                                                    this);

  pthread_mutex_init(&this->hokuyo_mutex_, NULL);
  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

ReactiveHokuyoAlgNode::~ReactiveHokuyoAlgNode(void)
{
  // [free dynamic memory]
}

void ReactiveHokuyoAlgNode::mainNodeThread(void)
{
  this->hokuyo_mutex_enter();
  if (flag_new_hokuyo_data_)
  {
    // [fill msg structures]
    flag_new_hokuyo_data_ = false;
    local_copy_of_input_scan_ = input_scan_;
    this->hokuyo_mutex_exit();

    this->alg_.incorporateSensorPoseInformation(local_copy_of_input_scan_, SENSOR_HEIGHT_, SENSOR_PITCH_DEG_ANGLE_,
                                                steering_angle_, real_3D_cloud_);

    this->alg_.eliminateSmallClusters(real_3D_cloud_, euclidean_association_threshold_, min_obstacle_radius_,
                                      obstacle_points_);

    this->alg_.filterNonObstaclePoints(obstacle_points_, z_threshold_, safety_width_, final_obstacles_);

    this->alg_.findClosestDistance(final_obstacles_, closest_obstacle_point_);

    // [publish messages]
    front_obstacle_distance_msg_.data = closest_obstacle_point_ - DISTANCE_FROM_SENSOR_TO_FRONT_;
    this->front_obstacle_distance_publisher_.publish(this->front_obstacle_distance_msg_);

    final_obstacles_.header.frame_id = local_copy_of_input_scan_.header.frame_id;
    final_obstacles_.header.stamp = local_copy_of_input_scan_.header.stamp;
    pointcloud_msg_ = final_obstacles_;
    this->pointcloud_publisher_.publish(this->pointcloud_msg_);

  }
  else
  {
    this->hokuyo_mutex_exit();
  }
}

/*  [subscriber callbacks] */

void ReactiveHokuyoAlgNode::hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  this->hokuyo_mutex_enter();

  //if(strcmp(msg->header.frame_id.c_string(), "Laser") == 0)
  if (msg->header.frame_id == "laser")
  {
    input_scan_ = *msg;

    //DEBUG!!
    //std::cout << "Hokuyo scan received!" << std::endl;
    if (msg == NULL)
      std::cout << std::endl << "Null pointer!!! in function hokuyo_callback!";

    flag_new_hokuyo_data_ = true;
  }

  this->hokuyo_mutex_exit();
}

void ReactiveHokuyoAlgNode::estimatedAckermannStateCB(
    const ackermann_msgs::AckermannDriveStamped& estimated_ackermann_state_msg)
{
  this->hokuyo_mutex_enter();

  steering_angle_ = estimated_ackermann_state_msg.drive.steering_angle;

  this->hokuyo_mutex_exit();
}

void ReactiveHokuyoAlgNode::hokuyo_mutex_enter(void)
{
  pthread_mutex_lock(&this->hokuyo_mutex_);
}

void ReactiveHokuyoAlgNode::hokuyo_mutex_exit(void)
{
  pthread_mutex_unlock(&this->hokuyo_mutex_);
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void ReactiveHokuyoAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;

  abs_lateral_safety_margin_ = config_.lateral_safety_margin;

  //Update the safety width
  safety_width_ = VEHICLE_WIDTH_ + (2.0 * abs_lateral_safety_margin_);

  z_threshold_ = config_.min_obstacle_height;

  euclidean_association_threshold_ = config_.euclidean_association_threshold;

  min_obstacle_radius_ = config_.min_obstacle_radius;
  this->alg_.unlock();
}

void ReactiveHokuyoAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < ReactiveHokuyoAlgNode > (argc, argv, "reactive_hokuyo_alg_node");
}
