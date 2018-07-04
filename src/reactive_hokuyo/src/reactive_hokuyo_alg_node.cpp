#include "reactive_hokuyo_alg_node.h"
#include "reactive_hokuyo_alg.h"

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

	closest_obstacle_point_ = IMPOSSIBLE_RANGE_VALUE_;

	max_velocity_recommendation_ = STOP_VEHICLE_;

	this->loop_rate_ = 500;

	// Init subscribers
	this->hokuyo_subscriber_ = this->public_node_handle_.subscribe ("/scan", 1, &ReactiveHokuyoAlgNode::hokuyo_callback, this);

	pthread_mutex_init (&this->hokuyo_mutex_, NULL);

	// Init publishers
	this->recommended_velocity_publisher_ = this->public_node_handle_.advertise
	      < std_msgs::Float32 > ("hokuyo_recommended_velocity", 1);

}

void ReactiveHokuyoAlgNode::mainNodeThread(void)
{
	this->hokuyo_mutex_enter();
	if(flag_new_hokuyo_data_)
	{
		flag_new_hokuyo_data_ = false;
		local_copy_of_input_scan_ = input_scan_;
		this->hokuyo_mutex_exit();

		this->alg_.incorporateSensorPoseInformation(local_copy_of_input_scan_, SENSOR_HEIGHT_, SENSOR_PITCH_DEG_ANGLE_, real_3D_cloud_);

		this->alg_.filterNonObstaclePoints(real_3D_cloud_, z_threshold_, obstacle_points_);

		this->alg_.eliminateSmallClusters(obstacle_points_, euclidean_association_threshold_, min_obstacle_radius_, final_obstacles_);

		this->alg_.findClosestDistance(final_obstacles_, closest_obstacle_point_);

		max_velocity_recommendation_ = ( closest_obstacle_point_ - safety_distance_to_stop_vehicle_ ) / time_to_reach_min_allowed_distance_;

		recommended_velocity_msg_.data = max_velocity_recommendation_;

		this->recommended_velocity_publisher_.publish (this->recommended_velocity_msg_);
	}else{
		this->hokuyo_mutex_exit();
	}
}


void ReactiveHokuyoAlgNode::hokuyo_callback (const sensor_msgs::LaserScan::ConstPtr& msg)
{
  this->hokuyo_mutex_enter ();

  if(strcmp(msg->header.frame_id, "Laser") == 0)
  {
	  input_scan_ = *msg;

	  //DEBUG!!
	  std::cout << "Hokuyo scan received!" << std::endl;
	  if (msg == NULL) std::cout << std::endl << "Null pointer!!! in function hokuyo_callback!";

	  flag_new_hokuyo_data_ = true;
  }

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
