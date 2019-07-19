/// Perception model for terrain following
/// Zhiang Chen, 7/2019

/*The MIT License (MIT)
Copyright (c) 2019 Zhiang Chen
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef Perception_Model_H_
#define Perception_Model_H_

#include <iostream> // for C++
#include <fstream>
#include <string>
#include <math.h>

#include <ros/ros.h> // for ROS
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/point_cloud.h> // for PCL
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h> // required for pcl::transformPointCloud


#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;

class Perception_Model
{
public:
	Perception_Model(ros::NodeHandle *nh, string pc_topic);
	

private:
	ros::NodeHandle nh_;
	string pc_topic_;
	tf::TransformListener tf_listener_;
	tf::StampedTransform tf_transform_;
	geometry_msgs::PoseStamped pose_;
	Eigen::Affine3d T_camera2base_;
	Eigen::Affine3d T_base2world_;
	Eigen::Affine3d T_camera2world_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_ptr_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclTransformed_ptr_;
	bool pc_got_;
	

	ros::Subscriber pc_sub_;
	ros::Subscriber pose_sub_;
	ros::Publisher pc_pub_;
	ros::Publisher pose_pub_;

	Eigen::Affine3d Posestamped2Affine3d_(geometry_msgs::PoseStamped stPose);	
	Eigen::Affine3d TF2Affine3d_(tf::StampedTransform sTf);
	geometry_msgs::Pose Affine3d2Pose_(Eigen::Affine3d affine);
	void transform_cloud_(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_ptr, string frame); 
	

	void pcCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);	
	void poseCallback(const geometry_msgs::PoseStamped pose);
};

#endif
