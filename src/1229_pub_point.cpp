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

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	bool filter_point = true;
	PointCloud::Ptr cloud (new PointCloud);
	PointCloud::Ptr cloud_out (new PointCloud);
	PointCloud::Ptr cloud_voxel (new PointCloud);
	PointCloud::Ptr cloud_pass_z(new PointCloud);
	//PointCloud::Ptr cloud_pass_y(new PointCloud);
	PointCloud::Ptr cloud_pass_x(new PointCloud);
	//PointCloud::Ptr cloud_indice(new PointCloud);
	PointCloud::Ptr cloud_filtered(new PointCloud);
	sensor_msgs::PointCloud2::Ptr cloud_pub(new sensor_msgs::PointCloud2);

	pcl::fromROSMsg(*input, *cloud);
	cloud->header.frame_id = input->header.frame_id;//"camera_depth_optical";
	cloud->header.stamp = ros::Time::now().toNSec();
	cloud->is_dense = false;

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
	cloud_out->header.frame_id = "base_link";
	cloud_out->header.stamp = ros::Time::now().toNSec();
	pcl_conversions::toPCL(ros::Time::now(), cloud_out->header.stamp);



	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_out);
	sor.setLeafSize (0.02f, 0.02f, 0.02f);
	sor.filter (*cloud_voxel);

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


	// pcl::PassThrough<pcl::PointXYZ> pass_y;
	// pass_y.setInputCloud(cloud_pass_z);
	// pass_y.setFilterFieldName("y");
	// pass_y.setFilterLimits(-0.505, 1.2);
	// pass_y.filter(*cloud_pass_y);

	if (filter_point && cloud_pass_x->points.size() > 5)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_z;
		pass_z.setInputCloud(cloud_pass_x);
		pass_z.setFilterFieldName("z");
		pass_z.setFilterLimits(0.0, 1.7);
		pass_z.filter(*cloud_pass_z);
	}
	else filter_point = false;
	/*}
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	size_t ind = 0;
	for (size_t t = 0; t < cloud_pass_x->points.size(); t++)
	{
	    if (cloud_pass_x->points[t].x > 0.28 && cloud_pass_x->points[t].x < 0.62 && cloud_pass_x->points[t].y > -0.59 && cloud_pass_x->points[t].y < -0.34)
	    {
	        inliers->indices.push_back(t);
	        ind++;
	    }

	}
	// Extract the inliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud_pass_x);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*cloud_indice);*/
	if (filter_point && cloud_pass_z->points.size() > 5)
	{
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> RadiusOR;
		RadiusOR.setInputCloud(cloud_pass_z);
		RadiusOR.setRadiusSearch(0.18);
		RadiusOR.setMinNeighborsInRadius (25);
		RadiusOR.filter (*cloud_filtered);

		pcl::toROSMsg(*cloud_filtered, *cloud_pub);
		cloud_pub->header.frame_id = "base_link";
		cloud_pub->header.stamp = input->header.stamp;//ros::Time::now();
		pub.publish(*cloud_pub);
		ROS_INFO("cloud_pub published!");
	}
	else
	{
		cloud_filtered->points.resize(1);
		cloud_filtered->points[0].x = 200;
		cloud_filtered->points[0].y = 200;
		cloud_filtered->points[0].z = 200;
		pcl::toROSMsg(*cloud_filtered, *cloud_pub);
		cloud_pub->header.frame_id = "base_link";
		cloud_pub->header.stamp = input->header.stamp;//ros::Time::now();
		pub.publish(*cloud_pub);
	}

	cloud->points.clear();
	cloud_out->points.clear();
	cloud_voxel->points.clear();
	cloud_pass_x->points.clear();
	// cloud_pass_y->points.clear();
	//cloud_indice->points.clear();
	cloud_pass_z->points.clear();
	cloud_filtered->points.clear();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh;
	listener = new( tf::TransformListener);
	pub = nh.advertise<sensor_msgs::PointCloud2> ("output_points", 1);
	//pub = nh.advertise<PointCloud> ("output_points", 1);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_points", 1, callback);
	ros::spin();
	return 0;
}
