#ifndef _reactive_hokuyo_alg_node_h_
#define _reactive_hokuyo_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>

class ReactiveHokuyoAlgNode : public algorithm_base::IriBaseAlgorithm<ReactiveHokuyoAlgorithm>
{
private:
    // Constants
	const float IMPOSSIBLE_RANGE_VALUE_ = -1.0;
	const float STOP_VEHICLE_          =  0.0;

	// Constant robot hardware constraints
	const float DISTANCE_FROM_SENSOR_TO_FRONT_ =  0.200;
	const float SENSOR_PITCH_DEG_ANGLE_        = 15.000;
	const float SENSOR_HEIGHT_                 =  0.650;

	const float VEHICLE_WIDTH_                 =  0.800;
	const float ABS_MAX_STEERING_DEG_ANGLE     = 30.000;
	const float MIN_OBSTACLE_HEIGHT_           =  0.075;

	// Input
	bool flag_new_hokuyo_data_;
	sensor_msgs::LaserScan input_scan_;
	sensor_msgs::LaserScan local_copy_of_input_scan_;

	// Point cloud to convert to 3D using the sensor pose
	pcl::PCLPointCloud2 real_3D_cloud_;
	pcl::PCLPointCloud2 obstacle_points_;
	pcl::PCLPointCloud2 final_obstacles_;

	// Configurable safety parameters
	float time_to_reach_min_allowed_distance_;
	float safety_distance_to_stop_vehicle_;

	float abs_lateral_safety_margin_;

	float z_threshold_;
	float safety_width_;

	float euclidean_association_threshold_;
	float min_obstacle_radius_;

	// Values to compute output
	float closest_obstacle_point_;

	// Node output
	float max_velocity_recommendation_;

    // [subscriber attributes]
    ros::Subscriber hokuyo_subscriber_;
    void hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t hokuyo_mutex_;
    void hokuyo_mutex_enter(void);
    void hokuyo_mutex_exit(void);

    // [publisher attributes]
    ros::Publisher recommended_velocity_publisher_;
    std_msgs::Float32 recommended_velocity_msg_;

public:

	ReactiveHokuyoNode(void);

	~ReactiveHokuyoNode(void);

protected:

   /**
	* \brief main node thread
	*
	* This is the main thread node function. Code written here will be executed
	* in every node loop while the algorithm is on running state. Loop frequency
	* can be tuned by modifying loop_rate attribute.
	*
	* Here data related to the process loop or to ROS topics (mainly data structs
	* related to the MSG and SRV files) must be updated. ROS publisher objects
	* must publish their data in this process. ROS client servers may also
	* request data to the corresponding server topics.
	*/
	void mainNodeThread(void);

   /**
	* \brief dynamic reconfigure server callback
	*
	* This method is called whenever a new configuration is received through
	* the dynamic reconfigure. The derivated generic algorithm class must
	* implement it.
	*
	* \param config an object with new configuration from all algorithm
	*               parameters defined in the config file.
	* \param level  integer referring the level in which the configuration
	*               has been changed.
	*/
	// void node_config_update(Config &config, uint32_t level);

   /**
	* \brief node add diagnostics
	*
	* In this abstract function additional ROS diagnostics applied to the
	* specific algorithms may be added.
	*/
	// void addNodeDiagnostics(void);

};



#endif
