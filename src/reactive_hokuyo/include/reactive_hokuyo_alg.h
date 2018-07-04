#ifndef _reactive_hokuyo_alg_h_
#define _reactive_hokuyo_alg_h_

class ReactiveHokuyoAlgorithm
{
private:


protected:

	pthread_mutex_t access_;

public:

	void incorporateSensorPoseInformation(sensor_msgs::LaserScan& local_copy_of_input_scan_,
											float SENSOR_HEIGHT_, float SENSOR_PITCH_DEG_ANGLE_,
											pcl::PCLPointCloud2& real_3D_cloud_);

	void filterNonObstaclePoints(pcl::PCLPointCloud2& real_3D_cloud_,
									float z_threshold_,
									pcl::PCLPointCloud2& obstacle_points_);

	void eliminateSmallClusters(pcl::PCLPointCloud2& obstacle_points_,
									float euclidean_association_threshold_, float min_obstacle_radius_,
									pcl::PCLPointCloud2& final_obstacles_);

	void findClosestDistance(pcl::PCLPointCloud2& final_obstacles_,
								float closest_obstacle_point_);

   /**
	* \brief define config type
	*
	* Define a Config type with the ReactiveHokuyoConfig. All driver implementations
	* will then use the same variable type Config.
	*/
	typedef blue_package::ReactiveHokuyoConfig Config;

   /**
	* \brief config variable
	*
	* This variable has all the driver parameters defined in the cfg config file.
	* Is updated everytime function config_update() is called.
	*/
	Config config_;

   /**
	* \brief constructor
	*
	* In this constructor parameters related to the specific driver can be
	* initalized. Those parameters can be also set in the openDriver() function.
	* Attributes from the main node driver class IriBaseDriver such as loop_rate,
	* may be also overload here.
	*/
	ReactiveHokuyoAlgorithm(void);

   /**
	* \brief Lock Algorithm
	*
	* Locks access to the Algorithm class
	*/
	void lock(void) { pthread_mutex_lock(&this->access_); };

   /**
	* \brief Unlock Algorithm
	*
	* Unlocks access to the Algorithm class
	*/
	void unlock(void) { pthread_mutex_unlock(&this->access_); };

   /**
	* \brief Tries Access to Algorithm
	*
	* Tries access to Algorithm
	*
	* \return true if the lock was adquired, false otherwise
	*/
	bool try_enter(void)
	{
	  if(pthread_mutex_trylock(&this->access_)==0)
		return true;
	  else
		return false;
	};

   /**
	* \brief config update
	*
	* In this function the driver parameters must be updated with the input
	* config variable. Then the new configuration state will be stored in the
	* Config attribute.
	*
	* \param new_cfg the new driver configuration state
	*
	* \param level level in which the update is taken place
	*/
	void config_update(Config& config, uint32_t level=0);

	// here define all tracking_vehicles_alg interface methods to retrieve and set
	// the driver parameters

   /**
	* \brief Destructor
	*
	* This destructor is called when the object is about to be destroyed.
	*
	*/
	~ReactiveHokuyoAlgorithm(void);

};

#endif
