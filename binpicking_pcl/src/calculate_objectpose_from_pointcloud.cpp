#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/common.h>
//#include <pcl/ModelCoefficients.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

#include "calculate_objectpose_from_pointcloud.h"


geometry_msgs::Vector3 calculate_objectpose_from_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud) {

//	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
#if 0
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(cloud);
//	while (!viewer.wasStopped())
//	{
//	}
#endif

	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);


#if 0
	// Niet nodig 
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);
#endif

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices); // No indices
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	colored_cloud = reg.getColoredCloud();

	ROS_INFO("Number of clusters/objects is equal to %i", clusters.size());

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	int cnt = 0;
	pcl::PointXYZ CenterPoint;
	pcl::PointXYZ CenterPointMin;
	// Find neraest object
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
	{	
		/* Create pointcloud from cluster */
		cloud_cluster->points.clear();
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); 
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;



		// Find centroid of the object
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud_cluster, centroid);

		// Find distance of the object
		pcl::PointXYZ minPt, maxPt;
		pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

		// Determinate neraest object from previos objects
		CenterPoint.x = centroid[0];
		CenterPoint.y = centroid[1];
		CenterPoint.z = minPt.z;
		if(!cnt){
			CenterPointMin = CenterPoint;
		}
		else{
			if(CenterPoint.z < CenterPointMin.z)
				CenterPointMin = CenterPoint;
		}
		// ROS_INFO("The middlepoint of object (%i): %f, %f, %f (x,y,z)", cnt, CenterPoint.x, CenterPoint.y, CenterPoint.z); 
		cnt++;
	}
	ROS_INFO("The centerpoint of the nearest object: %f, %f, %f (x,y,z)", CenterPointMin.x, CenterPointMin.y, CenterPointMin.z); 

	geometry_msgs::Vector3 result;
	
	result.x = CenterPointMin.x;
	result.y = CenterPointMin.y;
	result.z = CenterPointMin.z;

	return result;


#if 0
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
	}
#endif

}

