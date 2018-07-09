#include "velocity_recommender_alg.h"

VelocityRecommenderAlgorithm::VelocityRecommenderAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

VelocityRecommenderAlgorithm::~VelocityRecommenderAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void VelocityRecommenderAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// VelocityRecommenderAlgorithm Public API
