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

#ifndef _reactive_hokuyo_alg_h_
#define _reactive_hokuyo_alg_h_

#include <ReactiveHokuyoConfig.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>

//include reactive_hokuyo_alg main library

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class ReactiveHokuyoAlgorithm
{
  private:
	laser_geometry::LaserProjection projector_;
  protected:
   /**
    * \brief define config type
    *
    * Define a Config type with the ReactiveHokuyoConfig. All driver implementations
    * will then use the same variable type Config.
    */
    pthread_mutex_t access_;    

    // private attributes and methods

  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the ReactiveHokuyoConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_reactive_hokuyo::ReactiveHokuyoConfig Config;

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

    // here define all reactive_hokuyo_alg interface methods to retrieve and set
    // the driver parameters

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~ReactiveHokuyoAlgorithm(void);

	void incorporateSensorPoseInformation(sensor_msgs::LaserScan& local_copy_of_input_scan,
											float sensor_height, float sensor_pitch_deg_angle, float steering_deg_angle,
											sensor_msgs::PointCloud2& real_3D_cloud);

	void filterNonObstaclePoints(sensor_msgs::PointCloud2& real_3D_cloud,
									float z_threshold, float vehicle_width,
									sensor_msgs::PointCloud2& obstacle_points);

	void eliminateSmallClusters(sensor_msgs::PointCloud2& obstacle_points,
									float euclidean_association_threshold, float min_obstacle_radius,
									sensor_msgs::PointCloud2& final_obstacles);

	void findClosestDistance(sensor_msgs::PointCloud2& final_obstacles,
								float& closest_obstacle_point);
};

#endif
