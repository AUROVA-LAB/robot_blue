#include "reactive_hokuyo_alg.h"
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <laser_geometry/laser_geometry.h>
#include "pcl_ros/transforms.h"
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>


ReactiveHokuyoAlgorithm::ReactiveHokuyoAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

ReactiveHokuyoAlgorithm::~ReactiveHokuyoAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void ReactiveHokuyoAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// ReactiveHokuyoAlgorithm Public API
void ReactiveHokuyoAlgorithm::incorporateSensorPoseInformation(sensor_msgs::LaserScan& local_copy_of_input_scan_,
																float sensor_height, float sensor_pitch_deg_angle, float steering_deg_angle,
																sensor_msgs::PointCloud2& real_3D_cloud_)
{
	//std::cout << "incorporateSensorPoseInformation" << std::endl;

	// We convert the laser scan to 3D pointcloud
	projector_.projectLaser(local_copy_of_input_scan_, real_3D_cloud_);

	Eigen::Matrix <float, 4, 4> transform;

	// We include the sensor pitch and height
	float radians_pitch_angle = sensor_pitch_deg_angle * M_PI / 180.0;
	transform(0,0) = cos(radians_pitch_angle);
	transform(0,1) = 0.0;
	transform(0,2) = sin(radians_pitch_angle);
	transform(0,3) = 0.0;

	transform(1,0) = 0.0;
	transform(1,1) = 1.0;
	transform(1,2) = 0.0;
	transform(1,3) = 0.0;

	transform(2,0) = -1.0*sin(radians_pitch_angle);
	transform(2,1) = 0.0;
	transform(2,2) = cos(radians_pitch_angle);
	transform(2,3) = sensor_height;

	transform(3,0) = 0.0;
	transform(3,1) = 0.0;
	transform(3,2) = 0.0;
	transform(3,3) = 1.0;

	pcl_ros::transformPointCloud(transform,real_3D_cloud_, real_3D_cloud_);

	// We correct the angle to orient the data as if sensor were attached to the steering system
	float radians_steering_angle = steering_deg_angle * M_PI / 180.0;

	radians_steering_angle = -1*radians_steering_angle; // we need to correct it the opposite direction of the steering

	transform(0,0) = cos(radians_steering_angle);
	transform(0,1) = -1.0*sin(radians_steering_angle);
	transform(0,2) = 0.0;
	transform(0,3) = 0.0;

	transform(1,0) = sin(radians_steering_angle);;
	transform(1,1) = cos(radians_steering_angle);
	transform(1,2) = 0.0;
	transform(1,3) = 0.0;

	transform(2,0) = 0.0;
	transform(2,1) = 0.0;
	transform(2,2) = 1.0;
	transform(2,3) = 0.0;

	transform(3,0) = 0.0;
	transform(3,1) = 0.0;
	transform(3,2) = 0.0;
	transform(3,3) = 1.0;

	pcl_ros::transformPointCloud(transform,real_3D_cloud_, real_3D_cloud_);

}

void ReactiveHokuyoAlgorithm::filterNonObstaclePoints(sensor_msgs::PointCloud2& real_3D_cloud,
														float z_threshold, float vehicle_width,
														sensor_msgs::PointCloud2& obstacle_points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr aux (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(real_3D_cloud,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*input_cloud);

	//std::cout << "filterNonObstaclePoints" << std::endl;
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (input_cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_threshold, 10.0);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*aux);

	pass.setInputCloud (aux);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-1*vehicle_width/2.0, vehicle_width/2.0);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);

	pcl::PCLPointCloud2 output_cloud;
	toPCLPointCloud2 (*cloud_filtered, output_cloud);
	pcl_conversions::fromPCL (output_cloud, obstacle_points);

}

void ReactiveHokuyoAlgorithm::eliminateSmallClusters(sensor_msgs::PointCloud2& obstacle_points,
														float euclidean_association_threshold, float min_obstacle_radius,
														sensor_msgs::PointCloud2& final_obstacles)
{
	//std::cout << "eliminateSmallClusters" << std::endl;
/*
	  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	  pcl::PCLPointCloud2 cloudPCL;
	  pcl_conversions::toPCL(obstacle_points_, cloudPCL);
	  pcl::fromPCLPointCloud2(cloudPCL,*input_cloud);

	  // Creating the KdTree object for the search method of the extraction
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud(input_cloud);

	  std::vector<pcl::PointIndices> cluster_indices;

	  pcl::EuclideanClusterExtraction < pcl::PointXYZ > ec;
	  ec.setClusterTolerance(euclidean_association_threshold_);
	  //ec.setMinClusterSize(0);
	  //ec.setMaxClusterSize(max_num_points);
	  ec.setSearchMethod(tree);
	  ec.setInputCloud(input_cloud);
	  ec.extract(cluster_indices);

	  int j = 0;
	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	    {
	    	cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
	    }
	    j++;
	  }
*/
	final_obstacles = obstacle_points;
}

void ReactiveHokuyoAlgorithm::findClosestDistance(sensor_msgs::PointCloud2& final_obstacles,
													float& closest_obstacle_point)
{
	//std::cout << "findClosestDistance" << std::endl;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	  pcl::PCLPointCloud2 cloudPCL;
	  pcl_conversions::toPCL(final_obstacles, cloudPCL);
	  pcl::fromPCLPointCloud2(cloudPCL,*input_cloud);

	  float x = 0.0;
	  float y = 0.0;
	  float distance;
	  float min_distance = 10000.0;

	  for (size_t i = 0; i < input_cloud->points.size(); ++i)
	  {
		x = input_cloud->points[i].x;
		y = input_cloud->points[i].y;

		distance = sqrt(x*x + y*y);
		if(distance < min_distance)
		{
			min_distance = distance;
		}
	  }

	  closest_obstacle_point = min_distance;
}
