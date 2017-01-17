#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <tf/transform_listener.h>

//#include <boost/foreach.hpp>
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub;
tf::TransformListener*  listener;

void callback(const PointCloud::ConstPtr& cloud)
{
	bool filter_point = true;
	PointCloud::Ptr cloud_out (new PointCloud);
	PointCloud::Ptr cloud_voxel (new PointCloud);
	PointCloud::Ptr cloud_pass_z(new PointCloud);
	PointCloud::Ptr cloud_pass_x(new PointCloud);
	PointCloud::Ptr cloud_pub(new PointCloud);

    // transform cloud   
	tf::StampedTransform transform;
	try
	{
		listener->waitForTransform("/base_link", cloud->header.frame_id, ros::Time(0), ros::Duration(1.0));
		listener->lookupTransform("/base_link", cloud->header.frame_id,  ros::Time(0), transform);//"/camera_depth_optical"
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical\" to \"base_link\": %s", ex.what());
	}
	pcl_ros::transformPointCloud( *cloud, *cloud_out, transform );

    //  voxel downsample
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_out);
	sor.setLeafSize (0.02f, 0.02f, 0.02f);
	sor.filter (*cloud_voxel);

    //  filter X axis 
	if (cloud_voxel->points.size() < 5)
		filter_point = false;
	else
	{
		pcl::PassThrough<pcl::PointXYZ> pass_x;
		pass_x.setInputCloud(cloud_voxel);
		pass_x.setFilterFieldName("x");
		pass_x.setFilterLimits(0, 1.8);
		pass_x.filter(*cloud_pass_x);
	}

    //  filter Z axis 
	if (filter_point && cloud_pass_x->points.size() > 5)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_z;
		pass_z.setInputCloud(cloud_pass_x);
		pass_z.setFilterFieldName("z");
		pass_z.setFilterLimits(0.0, 1.7);
		pass_z.filter(*cloud_pass_z);
	}
	else filter_point = false;

    //  remove radius outlier and publish cloud
	if (filter_point && cloud_pass_z->points.size() > 5)
	{
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> RadiusOR;
		RadiusOR.setInputCloud(cloud_pass_z);
		RadiusOR.setRadiusSearch(0.18);
		RadiusOR.setMinNeighborsInRadius (25);
		RadiusOR.filter (*cloud_pub);

		cloud_pub->header.frame_id = "base_link";
		cloud_pub->header.stamp = cloud->header.stamp;//ros::Time::now();
		pub.publish(*cloud_pub);
		ROS_INFO("cloud_pub published!");
	}
	else
	{
		cloud_pub->height = cloud_pub->width = 1;
		cloud_pub->points.push_back(pcl::PointXYZ(200,200,200));
		cloud_pub->header.frame_id = "base_link";
		cloud_pub->header.stamp = cloud->header.stamp;//ros::Time::now();
		pub.publish(*cloud_pub);
	}

	cloud_out->points.clear();
	cloud_voxel->points.clear();
	cloud_pass_x->points.clear();
	cloud_pass_z->points.clear();
	cloud_pub->points.clear();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh;
	listener = new( tf::TransformListener);
	pub = nh.advertise<PointCloud> ("output_points", 1);
	ros::Subscriber sub = nh.subscribe<PointCloud>("input_points", 1, callback);
	ros::spin();
	return 0;
}
