#include "velocity_recommender_alg_node.h"

VelocityRecommenderAlgNode::VelocityRecommenderAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<VelocityRecommenderAlgorithm>()
{
  flag_new_data_ = false;
  //init class attributes if necessary
  hokuyo_front_obstacle_distance = OUT_OF_RANGE;
  velodyne_front_obstacle_distance_ = OUT_OF_RANGE;
  local_map_front_obstacle_distance_ = OUT_OF_RANGE;
  min_front_obstacle_distance_ = OUT_OF_RANGE;

  forward_velocity_recommendation_ = MAX_VELOCITY;

  velodyne_back_obstacle_distance_ = -1 * OUT_OF_RANGE;
  local_map_back_obstacle_distance_ = -1 * OUT_OF_RANGE;
  min_back_obstacle_distance_ = -1 * OUT_OF_RANGE;

  backward_velocity_recommendation_ = -1 * MAX_VELOCITY;

  time_to_reach_min_allowed_distance_ = 2.0;
  safety_distance_to_stop_vehicle_ = 0.25;

  this->loop_rate_ = 500.0; //in [Hz]

  // [init publishers]
  this->forward_recommended_velocity_publisher_ = this->public_node_handle_.advertise < std_msgs::Float32
      > ("forward_recommended_velocity", 1);

  this->backward_recommended_velocity_publisher_ = this->public_node_handle_.advertise < std_msgs::Float32
      > ("backward_recommended_velocity", 1);

  // [init subscribers]
  this->reactive_hokuyo_subscriber_ = this->public_node_handle_.subscribe(
      "/reactive_hokuyo_alg_node/front_obstacle_distance", 1, &VelocityRecommenderAlgNode::reactive_hokuyo_callback,
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
  this->velocity_recommender_mutex_enter();
  if (flag_new_data_)
  {
    flag_new_data_ = false;

    // Forward
    min_front_obstacle_distance_ = OUT_OF_RANGE;
    if (hokuyo_front_obstacle_distance < min_front_obstacle_distance_)
    {
      min_front_obstacle_distance_ = hokuyo_front_obstacle_distance;
    }

    if (velodyne_front_obstacle_distance_ < min_front_obstacle_distance_)
    {
      min_front_obstacle_distance_ = velodyne_front_obstacle_distance_;
    }

    if (local_map_front_obstacle_distance_ < min_front_obstacle_distance_)
    {
      min_front_obstacle_distance_ = local_map_front_obstacle_distance_;
    }

    // Backward
    min_back_obstacle_distance_ = -1 * OUT_OF_RANGE;
    if (velodyne_back_obstacle_distance_ > min_back_obstacle_distance_)
    {
      min_back_obstacle_distance_ = velodyne_back_obstacle_distance_;
    }

    if (local_map_back_obstacle_distance_ > min_back_obstacle_distance_)
    {
      min_back_obstacle_distance_ = local_map_back_obstacle_distance_;
    }

    // Computing output
    // Forward
    forward_velocity_recommendation_ = (min_front_obstacle_distance_ - safety_distance_to_stop_vehicle_)
        / time_to_reach_min_allowed_distance_;
    //std::cout << forward_velocity_recommendation_ << std::endl;
    if (forward_velocity_recommendation_ < 0.0)
    {
      forward_velocity_recommendation_ = 0.0;
      std::cout << "WARNING! attempted to set a negative forward velocity!" << std::endl;
    }

    if (forward_velocity_recommendation_ > MAX_VELOCITY)
      forward_velocity_recommendation_ = MAX_VELOCITY;

    // Backward
    backward_velocity_recommendation_ = (min_back_obstacle_distance_ + safety_distance_to_stop_vehicle_)
        / time_to_reach_min_allowed_distance_;

    if (backward_velocity_recommendation_ > 0.0)
    {
      backward_velocity_recommendation_ = 0.0;
      std::cout << "WARNING! attempted to set a positive backward velocity!" << std::endl;
    }

    if (backward_velocity_recommendation_ < -1 * MAX_VELOCITY)
      backward_velocity_recommendation_ = -1 * MAX_VELOCITY;

    // [fill msg structures]

    forward_recommended_velocity_msg_.data = forward_velocity_recommendation_;
    backward_recommended_velocity_msg_.data = backward_velocity_recommendation_;
    // [publish messages]
    this->forward_recommended_velocity_publisher_.publish(this->forward_recommended_velocity_msg_);
    this->backward_recommended_velocity_publisher_.publish(this->backward_recommended_velocity_msg_);

  }
  else
  {
    this->velocity_recommender_mutex_exit();
  }
}

/*  [subscriber callbacks] */
void VelocityRecommenderAlgNode::reactive_hokuyo_callback(const std_msgs::Float32::ConstPtr& msg)
{
  this->velocity_recommender_mutex_enter();

  hokuyo_front_obstacle_distance = msg->data;
  flag_new_data_ = true;
  //std::cout << "Reactive Hokuyo received!" << std::endl;

  this->velocity_recommender_mutex_exit();

}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void VelocityRecommenderAlgNode::velocity_recommender_mutex_enter(void)
{
  pthread_mutex_lock(&this->velocity_recommender_mutex_);
}

void VelocityRecommenderAlgNode::velocity_recommender_mutex_exit(void)
{
  pthread_mutex_unlock(&this->velocity_recommender_mutex_);
}

void VelocityRecommenderAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;

  time_to_reach_min_allowed_distance_ = config_.safety_time;
  safety_distance_to_stop_vehicle_ = config_.safety_distance;

  this->alg_.unlock();
}

void VelocityRecommenderAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < VelocityRecommenderAlgNode > (argc, argv, "velocity_recommender_alg_node");
}
