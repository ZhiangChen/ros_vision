#include <terrain_following/perception_model_cpp.h>
#include <ros/ros.h>
#include <string>

using namespace std;

Perception_Model::Perception_Model(ros::NodeHandle *nh, string pc_topic)
{
	nh_ = *nh;
	pc_topic_ = pc_topic;
	pc_sub_ = nh_.subscribe(pc_topic_, 1, &Perception_Model::pcCallback, this);
	pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/world_point_cloud", 1, true);

	ROS_INFO("Peception Model Initialized!");
}

void Perception_Model::pcCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
	sensor_msgs::PointCloud2 output;
	output = *cloud;
	pc_pub_.publish(output);
}
