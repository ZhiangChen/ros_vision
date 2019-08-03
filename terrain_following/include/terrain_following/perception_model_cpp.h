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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/server/simple_action_server.h> // customized ROS messages
#include <terrain_following/terrainAction.h>


#include <pcl_ros/point_cloud.h> // for PCL
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h> // required for pcl::transformPointCloud
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> PC_POSE_POLICY;

#define BOX_X 0.5
#define BOX_Y 0.5
#define BOX_POINTS_THRES 10
#define Queue_Size 10

class Perception_Model
{
public:
	Perception_Model(ros::NodeHandle *nh, string pc_topic);
	Perception_Model(ros::NodeHandle *nh);
	double get_terrain_height(double x, double y);

protected:
	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_; // tf
	tf::StampedTransform tf_transform_;
	geometry_msgs::PoseStamped pose_;
	Eigen::Affine3d T_camera2base_;
	Eigen::Affine3d T_base2world_;
	Eigen::Affine3d T_camera2world_;

	string pc_topic_; // pointcloud
	bool pc_got_;
	bool Tf_got_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_rgbd_ptr_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_ptr_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclTransformed_ptr_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr boxTransformed_ptr_;
	pcl::PassThrough<pcl::PointXYZRGB> box_filter_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformed_xyz_ptr_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boxTransformed_xyz_ptr_;
	Eigen::Vector4f box_centroid_;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_;
	sensor_msgs::PointCloud2 cloud_;


	Eigen::Vector3f no_translation_;
	Eigen::Vector3f no_roation_;

    terrain_following::terrainFeedback feedback_; // action
    terrain_following::terrainResult result_;

	ros::Subscriber pc_sub_; // service
	ros::Subscriber pose_sub_;
	ros::Publisher pc_pub_;
	ros::Publisher pose_pub_;
	actionlib::SimpleActionServer<terrain_following::terrainAction> as_;

	message_filters::Subscriber<sensor_msgs::PointCloud2> *mf_pc_sub_; // message filter
	message_filters::Subscriber<geometry_msgs::PoseStamped> *mf_pose_sub_;
	message_filters::Synchronizer<PC_POSE_POLICY> *pose_pc_sync_;

	Eigen::Affine3d Posestamped2Affine3d_(geometry_msgs::PoseStamped stPose); // function
	Eigen::Affine3d TF2Affine3d_(tf::StampedTransform sTf);
	geometry_msgs::Pose Affine3d2Pose_(Eigen::Affine3d affine);
	void transform_cloud_(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_ptr, string frame);
	void box_filtering_(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, vector<int> &indices, double x, double y);

	void pcCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);	// callback
	void poseCallback(const geometry_msgs::PoseStamped pose);
	void executeCB(const terrain_following::terrainGoalConstPtr &goal);
	void mfCallback(const sensor_msgs::PointCloud2ConstPtr &cloud, const geometry_msgs::PoseStampedConstPtr &pose);
};

#endif
