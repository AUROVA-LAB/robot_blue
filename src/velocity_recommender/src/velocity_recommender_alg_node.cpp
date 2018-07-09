#include "velocity_recommender_alg_node.h"

VelocityRecommenderAlgNode::VelocityRecommenderAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<VelocityRecommenderAlgorithm>()
{
	flag_new_data_ = false;
  //init class attributes if necessary
  forward_hokuyo_recommended_velocity_    = MAX_VELOCITY;
  forward_velodyne_recommended_velocity_  = MAX_VELOCITY;
  forward_local_map_recommended_velocity_ = MAX_VELOCITY;

  forward_velocity_recommendation_        = MAX_VELOCITY;

  backward_velodyne_recommended_velocity_ = MAX_VELOCITY;
  backward_local_map_recommended_velocity_= MAX_VELOCITY;

  backward_velocity_recommendation_       = MAX_VELOCITY;

  this->loop_rate_ = 500.0;//in [Hz]

  // [init publishers]
  this->forward_recommended_velocity_publisher_ = this->public_node_handle_.advertise
	      < std_msgs::Float32 > ("forward_recommended_velocity", 1);

  this->backward_recommended_velocity_publisher_ = this->public_node_handle_.advertise
	      < std_msgs::Float32 > ("backward_recommended_velocity", 1);
  
  // [init subscribers]
  this->reactive_hokuyo_subscriber_ = this->public_node_handle_.subscribe ("/reactive_hokuyo_alg_node/hokuyo_recommended_velocity",
		  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  1, &VelocityRecommenderAlgNode::reactive_hokuyo_callback,
																			  this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

VelocityRecommenderAlgNode::~VelocityRecommenderAlgNode(void)
{
  // [free dynamic memory]
}

void VelocityRecommenderAlgNode::mainNodeThread(void)
{
	this->velocity_recommender_mutex_enter ();
	if(flag_new_data_)
	{
		flag_new_data_ = false;

	  // Forward
	  forward_velocity_recommendation_ = MAX_VELOCITY;
	  if(forward_hokuyo_recommended_velocity_ < forward_velocity_recommendation_)
	  {
		  forward_velocity_recommendation_ = forward_hokuyo_recommended_velocity_;
	  }

	  if(forward_velodyne_recommended_velocity_ < forward_velocity_recommendation_)
	  {
		  forward_velocity_recommendation_ = forward_velodyne_recommended_velocity_;
	  }

	  if(forward_local_map_recommended_velocity_ < forward_velocity_recommendation_)
	  {
		  forward_velocity_recommendation_ = forward_local_map_recommended_velocity_;
	  }

	  // Backward

	  backward_velocity_recommendation_ = MAX_VELOCITY;
	  if(backward_velodyne_recommended_velocity_ < backward_velocity_recommendation_)
	  {
		  backward_velocity_recommendation_ = backward_velodyne_recommended_velocity_;
	  }

	  if(backward_local_map_recommended_velocity_ < backward_velocity_recommendation_)
	  {
		  backward_velocity_recommendation_ = backward_local_map_recommended_velocity_;
	  }
	  // [fill msg structures]
	  forward_recommended_velocity_msg_.data = forward_velocity_recommendation_;
	  backward_recommended_velocity_msg_.data = backward_velocity_recommendation_;
	  // [publish messages]
	  this->forward_recommended_velocity_publisher_.publish (this->forward_recommended_velocity_msg_);
	  this->backward_recommended_velocity_publisher_.publish (this->backward_recommended_velocity_msg_);

	}else{
	  this->velocity_recommender_mutex_exit ();
	}
}

/*  [subscriber callbacks] */
void VelocityRecommenderAlgNode::reactive_hokuyo_callback(const std_msgs::Float32::ConstPtr& msg)
{
	this->velocity_recommender_mutex_enter ();

	forward_hokuyo_recommended_velocity_ = msg->data;
	flag_new_data_ = true;
	//std::cout << "Reactive Hokuyo received!" << std::endl;

	this->velocity_recommender_mutex_exit ();

}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void VelocityRecommenderAlgNode::velocity_recommender_mutex_enter (void)
{
  pthread_mutex_lock (&this->velocity_recommender_mutex_);
}

void VelocityRecommenderAlgNode::velocity_recommender_mutex_exit (void)
{
  pthread_mutex_unlock (&this->velocity_recommender_mutex_);
}

void VelocityRecommenderAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void VelocityRecommenderAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<VelocityRecommenderAlgNode>(argc, argv, "velocity_recommender_alg_node");
}
