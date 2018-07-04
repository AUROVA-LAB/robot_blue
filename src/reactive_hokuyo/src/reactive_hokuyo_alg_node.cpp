#include "reactive_hokuyo_alg_node.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/segmentation/sac_segmentation.h>


ReactiveHokuyoAlgNode::ReactiveHokuyoAlgNode (void) :
    algorithm_base::IriBaseAlgorithm<ReactiveHokuyoAlgorithm> ()
{
	flag_new_hokuyo_data_ = false;

	time_to_reach_min_allowed_distance_ = 2.0;
	safety_distance_to_stop_vehicle_    = 0.25;

	z_threshold_ = -1.0 * ( SENSOR_HEIGHT_ - MIN_OBSTACLE_HEIGHT_ );

	abs_lateral_safety_margin_ = 0.2;

	safety_width_ = VEHICLE_WIDTH_ + ( 2.0 * abs_lateral_safety_margin_ );

	euclidean_association_threshold_ = 0.10;
	min_obstacle_radius_             = 0.02;

	closer_obstacle_point_ = IMPOSSIBLE_RANGE_VALUE_;

	max_velocity_recommendation_ = STOP_VEHICLE_;

}

void ReactiveHokuyoAlgNode::mainNodeThread(void)
{


}


void ReactiveHokuyoAlgNode::hokuyo_callback (const sensor_msgs::LaserScan::ConstPtr& msg)
{
  this->hokuyo_mutex_enter ();
  input_scan_ = *msg;

  //DEBUG!!
  if (msg == NULL) std::cout << std::endl << "Null pointer!!! in function hokuyo_callback!";

  flag_new_hokuyo_data_ = true;

  //number_of_point_clouds_received++;
  //std::cout << "Total velodyne msgs = " << number_of_point_clouds_received << std::endl;

  this->hokuyo_mutex_exit ();
}

void ReactiveHokuyoAlgNode::hokuyo_mutex_enter (void)
{
  pthread_mutex_lock (&this->hokuyo_mutex_);
}

void ReactiveHokuyoAlgNode::hokuyo_mutex_exit (void)
{
  pthread_mutex_unlock (&this->hokuyo_mutex_);
}
