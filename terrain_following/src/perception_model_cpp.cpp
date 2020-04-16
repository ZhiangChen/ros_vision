#include <terrain_following/perception_model_cpp.h>
#include <ros/ros.h>
#include <string>

using namespace std;


Perception_Model::Perception_Model(ros::NodeHandle *nh): nh_(*nh), pcl_rgbd_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>), pcl_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>), pclTransformed_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>), pclTransformed_ptr_copy_(new pcl::PointCloud<pcl::PointXYZRGB>), pclTransformed_xyz_ptr_(new pcl::PointCloud<pcl::PointXYZ>), boxTransformed_xyz_ptr_(new pcl::PointCloud<pcl::PointXYZ>), boxTransformed_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>), as_(*nh, "terrain_lookup", boost::bind(&Perception_Model::executeCB, this, _1), false)
{
	pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/world_point_cloud", 1, true);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/posestamped", 1, true); // for debug
	path_pub_ = nh_.advertise<nav_msgs::Path>("/terrain_following_path", 1, true);

	mf_pc_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/r200/depth/points", Queue_Size);
	mf_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "mavros/local_position/pose", Queue_Size);
	pose_pc_sync_ = new message_filters::Synchronizer<PC_POSE_POLICY>(PC_POSE_POLICY(Queue_Size), *mf_pc_sub_, *mf_pose_sub_);
	pose_pc_sync_->registerCallback(boost::bind(&Perception_Model::mfCallback, this, _1, _2));


	ROS_INFO("checking tf from camera to base_link ...");
	bool tf_exists = false;
	while ((!tf_exists) && (ros::ok()))
	{
		tf_exists = true;
		try
		{
			tf_listener_.lookupTransform("base_link", "r200", ros::Time(0), tf_transform_);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			ROS_INFO("retrying ...");
			tf_exists = false;
		}
	}
	ROS_INFO("tf is good.");
	T_camera2base_ = TF2Affine3d_(tf_transform_);
/*
	// trying to initialize T_camera2base_ from pose; for tuning transformation parameters
	Eigen::Vector3d Oe;
    Oe(0) = 0.1;
    Oe(1) = 0;
    Oe(2) = -0.01;
    Eigen::Quaterniond q;
    q.x() = -0.6743797;
    q.y() = 0.6743797;
    q.z() = -0.2126311;
    q.w() = 0.2126311;
	Eigen::Matrix3d Re(q);

    T_camera2base_.linear() = Re;
    T_camera2base_.translation() = Oe;
*/

	pc_got_ = false;
	Tf_got_ = false;
	no_translation_ << 0,0,0;
	no_rotation_ << 0,0,0;
	g_path_.header.frame_id = "map";
	as_.start(); //start the server running
	ROS_INFO("action service starts.");

	ROS_INFO("Peception Model Initialized!");
}



void Perception_Model::mfCallback(const sensor_msgs::PointCloud2ConstPtr &cloud, const geometry_msgs::PoseStampedConstPtr &pose)
{
	//double t = ros::Time::now().toSec();
	//cout << endl << t << endl;
	cloud_ = *cloud;
	pose_ = *pose;
	pcl::fromROSMsg(cloud_, *pcl_ptr_);

	T_base2world_ = Posestamped2Affine3d_(pose_);
	T_camera2world_ = T_base2world_ * T_camera2base_;
	Eigen::Affine3f A = T_camera2world_.cast<float>(); // need to convert to Eigen::Affine3f, which is compatible with pointcloud assignment

	pcl::transformPointCloud(*pcl_ptr_, *pclTransformed_ptr_, A);
	pclTransformed_ptr_->header.frame_id = "map";

	pcl::copyPointCloud(*pclTransformed_ptr_, *pclTransformed_ptr_copy_);

	pose_.pose = Affine3d2Pose_(T_camera2world_); // for debug
	pose_pub_.publish(pose); // for debug
	sensor_msgs::PointCloud2 output; // for debug
	pcl::toROSMsg(*pclTransformed_ptr_, output); // for debug
	pc_pub_.publish(output); // for debug

	//t = ros::Time::now().toSec();
	//cout << t << endl;

	g_path_.poses.push_back(pose_);
	path_pub_.publish(g_path_);
	pc_got_ = true;
}


void Perception_Model::executeCB(const terrain_following::terrainGoalConstPtr &goal)
{

	double x = goal->x;
	double y = goal->y;
	double z = goal->z;
	double relative_height = goal->relative_height;
	if (pc_got_)
	{

		pcl::CropBox<pcl::PointXYZRGB> box_crop;
		box_crop.setMin(Eigen::Vector4f(x - BOX_X/2.0, y - BOX_Y/2.0, -10000, 1.0));
		box_crop.setMax(Eigen::Vector4f(x + BOX_X/2.0, y + BOX_Y/2.0, 10000, 1.0));
		box_crop.setTranslation(no_translation_);
		box_crop.setRotation(no_rotation_);
		//Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		//box_crop.setTransform(transform);
		box_crop.setInputCloud(pclTransformed_ptr_copy_);
		box_crop.filter(*boxTransformed_ptr_);

		if (boxTransformed_ptr_->size() > BOX_POINTS_THRES)
		{

			//cout << boxTransformed_ptr_->size() <<endl;
			pcl::compute3DCentroid(*boxTransformed_ptr_, box_centroid_pcl_);
			//box_centroid_ = compute_centroid_(boxTransformed_ptr_, z);
			//cout << box_centroid_[0] <<" " << box_centroid_[1] << " " << box_centroid_[2]<<endl;

			//sensor_msgs::PointCloud2 output; // for debug
			//pcl::toROSMsg(*boxTransformed_ptr_, output); // for debug
			//pc_pub_.publish(output); // for debug

			result_.z = box_centroid_pcl_[2] + relative_height;
			result_.got_terrain = true;
			as_.setSucceeded(result_);
		}
		else
		{
			result_.z = z;
			result_.got_terrain = false;
			as_.setSucceeded(result_);
		}
	}
	else
	{
		result_.z = z;
		result_.got_terrain = false;
		as_.setSucceeded(result_);
	}
}


Eigen::Affine3d Perception_Model::Posestamped2Affine3d_(geometry_msgs::PoseStamped stPose)
{
    Eigen::Affine3d affine;
    geometry_msgs::Pose pose = stPose.pose;
    Eigen::Vector3d Oe;
    Oe(0) = pose.position.x;
    Oe(1) = pose.position.y;
    Oe(2) = pose.position.z;

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;
    affine.translation() = Oe;
    return affine;
}

Eigen::Affine3d Perception_Model::TF2Affine3d_(tf::StampedTransform sTf)
{
	Eigen::Affine3d affine;
	tf::Transform Tf(sTf.getBasis(), sTf.getOrigin()); // get tf from stamped tf

	for (int i = 0; i < 3; i++) {
        affine.matrix()(i, 3) = Tf.getOrigin()[i]; //copy the origin from tf to Eigen
        for (int j = 0; j < 3; j++) {
            affine.matrix()(i, j) = Tf.getBasis()[i][j]; //and copy 3x3 rotation matrix
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        affine.matrix()(3, col) = 0;
    affine.matrix()(3, 3) = 1;
    return affine;
}

geometry_msgs::Pose Perception_Model::Affine3d2Pose_(Eigen::Affine3d affine)
{
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = affine.translation();
    Re = affine.linear();
    //cout<<"affine R: "<<endl;
    //cout<<Re<<endl;

    Eigen::Quaterniond q(Re); // convert rotation matrix Re to a quaternion, q
    //cout<<"q: "<<q.x()<<", "<<q.y()<<", "<<q.z()<<", "<<q.w()<<endl;
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}



