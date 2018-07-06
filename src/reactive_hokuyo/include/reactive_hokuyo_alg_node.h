// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _reactive_hokuyo_alg_node_h_
#define _reactive_hokuyo_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "reactive_hokuyo_alg.h"
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include "std_msgs/Float32.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class ReactiveHokuyoAlgNode : public algorithm_base::IriBaseAlgorithm<ReactiveHokuyoAlgorithm>
{
private:
    // Constants
	const float IMPOSSIBLE_RANGE_VALUE_ = -1.0;
	const float STOP_VEHICLE_          =  0.0;

	// Constant robot hardware constraints
	const float DISTANCE_FROM_SENSOR_TO_FRONT_ =  0.150;
	const float SENSOR_PITCH_DEG_ANGLE_        = 15.000;
	const float SENSOR_HEIGHT_                 =  0.650;

	const float VEHICLE_WIDTH_                 =  0.800;
	const float ABS_MAX_STEERING_DEG_ANGLE     = 30.000;
	const float MIN_OBSTACLE_HEIGHT_           =  0.350;

	// Input
	bool flag_new_hokuyo_data_;
	sensor_msgs::LaserScan input_scan_;
	sensor_msgs::LaserScan local_copy_of_input_scan_;

	// Point cloud to convert to 3D using the sensor pose
	sensor_msgs::PointCloud2 real_3D_cloud_;
	sensor_msgs::PointCloud2 obstacle_points_;
	sensor_msgs::PointCloud2 final_obstacles_;

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
	float steering_angle_;

	// Node output
	float max_velocity_recommendation_;

    // [publisher attributes]
    ros::Publisher recommended_velocity_publisher_;
    std_msgs::Float32 recommended_velocity_msg_;

    ros::Publisher pointcloud_publisher_;
    sensor_msgs::PointCloud2 pointcloud_msg_;

	// [subscriber attributes]
    ros::Subscriber hokuyo_subscriber_;
    void hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::Subscriber ackermann_subscriber_;
    void estimatedAckermannStateCB(const ackermann_msgs::AckermannDriveStamped& estimated_ackermann_state_msg);

    pthread_mutex_t hokuyo_mutex_;
    void hokuyo_mutex_enter(void);
    void hokuyo_mutex_exit(void);


   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    ReactiveHokuyoAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~ReactiveHokuyoAlgNode(void);

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
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
