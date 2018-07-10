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
	std::cout << "incorporateSensorPoseInformation" << std::endl;

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
	std::cout << "filterNonObstaclePoints" << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr aux (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(real_3D_cloud,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*input_cloud);

	//std::cout << "filterNonObstaclePoints" << std::endl;
	pcl::PassThrough<pcl::PointXYZRGB> pass;
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

	//Uncomment to bypass
    //obstacle_points = real_3D_cloud;

}

void ReactiveHokuyoAlgorithm::eliminateSmallClusters(sensor_msgs::PointCloud2& obstacle_points,
														float euclidean_association_threshold, float min_obstacle_radius,
														sensor_msgs::PointCloud2& final_obstacles)
{
	std::cout << "eliminateSmallClusters" << std::endl;

	if(!obstacle_points.data.empty())
	{
		  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		  pcl::PCLPointCloud2 cloudPCL;
		  pcl_conversions::toPCL(obstacle_points, cloudPCL);
		  pcl::fromPCLPointCloud2(cloudPCL,*input_cloud);

		  // Creating the KdTree object for the search method of the extraction
		  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
		  tree->setInputCloud(input_cloud);

		  std::vector<pcl::PointIndices> cluster_indices;

		  pcl::EuclideanClusterExtraction < pcl::PointXYZRGB > ec;
		  ec.setClusterTolerance(euclidean_association_threshold);
		  //ec.setMinClusterSize(0);
		  //ec.setMaxClusterSize(max_num_points);
		  ec.setSearchMethod(tree);
		  ec.setInputCloud(input_cloud);
		  ec.extract(cluster_indices);

		  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		  int cluster_num = 0;
		  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		  {
			cluster_num++;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
			float x_accum = 0.0;
			float y_accum = 0.0;
			float z_accum = 0.0;
			int   count   = 0;
			std::cout<<"Cluster num = " << cluster_num << std::endl;
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				pcl::PointXYZRGB point;
				//cloud_cluster->points.push_back (input_cloud->points[*pit]);
				point.x = input_cloud->points[*pit].x;
				point.y = input_cloud->points[*pit].y;
				point.z = input_cloud->points[*pit].z;

				switch(cluster_num)
				{
					case 1:
						point.r = 255;
						point.g = 0;
						point.b = 0;
						break;
					case 2:
						point.r = 0;
						point.g = 255;
						point.b = 0;
						break;
					case 3:
						point.r = 0;
						point.g = 0;
						point.b = 255;
						break;
					default:
						point.r = 255;
						point.g = 255;
						point.b = 255;
				}

				cloud_cluster->points.push_back (point);

				x_accum += point.x;
				y_accum += point.y;
				z_accum += point.z;
				count++;
			}
			if(count > 0)
			{
				float x_centroid = x_accum / count;
				float y_centroid = y_accum / count;
				float z_centroid = z_accum / count;
				float max_distance = 0.0;

				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				{
					float dx = x_centroid - input_cloud->points[*pit].x;
					float dy = y_centroid - input_cloud->points[*pit].y;
					float dz = z_centroid - input_cloud->points[*pit].z;

					float distance = sqrt(dx*dx + dy*dy + dz*dz);
					if(distance > max_distance) max_distance = distance;
				}

				if( max_distance > min_obstacle_radius)
				{
					*filtered_cloud += *cloud_cluster;
				}else{
					std::cout << "Cluster discarded, radius = " << max_distance << std::endl;
				}
			}else{
				std::cout << "Warning! in ReactiveHokuyoAlgorithm::eliminateSmallClusters, cluster with zero points!!" << std::endl;
			}
			if(cluster_num == 3) cluster_num = 0;
		  }

		  pcl::PCLPointCloud2 cloud;
		  toPCLPointCloud2 (*filtered_cloud, cloud);
		  pcl_conversions::fromPCL (cloud, final_obstacles);
	}else{
	  final_obstacles = obstacle_points;
	}
}

void ReactiveHokuyoAlgorithm::findClosestDistance(sensor_msgs::PointCloud2& final_obstacles,
													float& closest_obstacle_point)
{
	  std::cout << "findClosestDistance" << std::endl;
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

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
