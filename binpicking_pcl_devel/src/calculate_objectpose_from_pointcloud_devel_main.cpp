#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../../binpicking_pcl/src/calculate_objectpose_from_pointcloud.h"

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
  // Create a container for the data.

#if 0
pcl::PointCloud<pcl::PointXYZ> point_cloud;
 
pcl::fromPCLPointCloud2( point_cloud2, point_cloud);
 
pcl::toPCLPointCloud2(point_cloud, point_cloud2);
#endif

	pcl::PCLPointCloud2* point_cloud2 = new pcl::PCLPointCloud2;
	pcl_conversions::toPCL(*input_cloud, *point_cloud2);  // From ROS-PCL to PCL2

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*point_cloud2, *point_cloud); // From PCL2 to PCL


	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_Pointcloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	calculate_objectpose_from_pointcloud(point_cloud, colored_Pointcloud);

	pcl::PCLPointCloud2* colored_Pointcloud2 = new pcl::PCLPointCloud2;
	pcl::toPCLPointCloud2(*colored_Pointcloud, *colored_Pointcloud2); // From PCL to PCL2

	sensor_msgs::PointCloud2Ptr output_cloud(new sensor_msgs::PointCloud2);// = new sensor_msgs::PointCloud2;
	pcl_conversions::fromPCL(*colored_Pointcloud2, *output_cloud);  // From PCL2 to ROS-PCL

	output_cloud->header = input_cloud->header;

	// Publish the data.
	pub.publish (output_cloud);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "binpicking_pcl_devel");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);


  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/color/debug", 1);


  ROS_INFO("Ready to calculate pose in development.");

  // Spin
  ros::spin ();
}



