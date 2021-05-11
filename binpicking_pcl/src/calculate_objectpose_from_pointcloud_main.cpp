#include "ros/ros.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloud.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloudRequest.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloudResponse.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
//#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//Image message
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "calculate_objectpose_from_pointcloud.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>


ros::Publisher image_pub; //image message publisher
sensor_msgs::Image image; //cache the image message


bool CalculateObjectposeFromPointcloud(binpicking_msgs::CalculateObjectposeFromPointcloud::Request  &request,
         binpicking_msgs::CalculateObjectposeFromPointcloud::Response &response)
{

	ROS_INFO("CalculateObjectposeFromPointcloud start");
	
	pcl::PCLPointCloud2* point_cloud2 = new pcl::PCLPointCloud2;
	pcl_conversions::toPCL(request.pointcloud, *point_cloud2);  // From ROS-PCL to PCL2

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*point_cloud2, *point_cloud); // From PCL2 to PCL


	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_Pointcloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	geometry_msgs::Vector3 object_position = calculate_objectpose_from_pointcloud(point_cloud, colored_Pointcloud);

		
	geometry_msgs::TransformStamped static_transformStamped;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	ROS_INFO("add static transform to object");
	/* add static transform to object */
	static_transformStamped.header.stamp = ros::Time::now();
	static_transformStamped.header.frame_id = "camera_depth_optical_frame";
	static_transformStamped.child_frame_id = "object_to_grasp";

	static_transformStamped.transform.translation = object_position;
	static_transformStamped.transform.rotation.x = 0;
	static_transformStamped.transform.rotation.y = 0;
	static_transformStamped.transform.rotation.z = 0;
	static_transformStamped.transform.rotation.w = 1.0;
	static_broadcaster.sendTransform(static_transformStamped);

	/* wait a while */
	ros::Duration(1.0).sleep();
	ROS_INFO("get object pose relative to world ");
	/* get object pose relative to world */
	try{
		geometry_msgs::TransformStamped transformStamped;
		geometry_msgs::PoseStamped poseStamped;
		poseStamped.pose.orientation.x = 0;
		poseStamped.pose.orientation.y = 0;
		poseStamped.pose.orientation.z = 0;
		poseStamped.pose.orientation.w = 1.0;
		transformStamped = tfBuffer.lookupTransform("base_link", "object_to_grasp", ros::Time(0));
		tf2::doTransform(poseStamped, poseStamped, transformStamped); // object_to_grasp is the PoseStamped I want to transform

		static tf2_ros::StaticTransformBroadcaster static_broadcaster;
		geometry_msgs::TransformStamped static_transformStamped;

		static_transformStamped.header.stamp = ros::Time::now();
		static_transformStamped.header.frame_id = "base_link";
		static_transformStamped.child_frame_id = "object_to_grasp_capture";
		static_transformStamped.transform.translation.x = poseStamped.pose.position.x;
		static_transformStamped.transform.translation.y = poseStamped.pose.position.y;
		static_transformStamped.transform.translation.z = poseStamped.pose.position.z;

		static_transformStamped.transform.rotation = poseStamped.pose.orientation;
		static_broadcaster.sendTransform(static_transformStamped);

		response.success = true;
		response.object_pose = poseStamped;

	}
	catch (tf2::TransformException &ex) {
		ROS_ERROR("Error lookupTransform.");
		response.success = false;
	}


	ROS_INFO("CalculateObjectposeFromPointcloud exit");
	/* return object pose in response*/
	return true;
}


int main(int argc, char **argv)
{


	ros::init(argc, argv, "calculate_object_pose_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("calculate_object_pose", CalculateObjectposeFromPointcloud);

	image_pub = n.advertise<sensor_msgs::Image> ("cloud_image", 30);

	ROS_INFO("Ready to calculate pose.");
	ros::spin();

	return 0;
}
