#include <terrain_following/perception_model_cpp.h>
#include <ros/ros.h>
#include <string>

using namespace std;


Perception_Model::Perception_Model(ros::NodeHandle *nh, string pc_topic): nh_(*nh), pc_topic_(pc_topic), pcl_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>), pclTransformed_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	pc_sub_ = nh_.subscribe(pc_topic_, 1, &Perception_Model::pcCallback, this);
	pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/world_point_cloud", 1, true);
	pose_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &Perception_Model::poseCallback, this);
	//pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/posestamped", 1, true); // for debug
	
	
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

	pc_got_ = false;


	ROS_INFO("Peception Model Initialized!");
}

void Perception_Model::pcCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
	pc_got_ = true;
	pcl::fromROSMsg(*cloud, *pcl_ptr_);
}

void Perception_Model::poseCallback(const geometry_msgs::PoseStamped pose)
{
	pose_ = pose;
	T_base2world_ = Posestamped2Affine3d_(pose_);
	T_camera2world_ = T_base2world_ * T_camera2base_; 
	//pose_.pose = Affine3d2Pose_(T_camera2world_); // for debug
	//pose_pub_.publish(pose_); // for debug
	
	if (pc_got_)
	{
		sensor_msgs::PointCloud2 output;
		Eigen::Affine3f A = T_camera2world_.cast<float>(); // need to convert to Eigen::Affine3f, which is compatible with pointcloud assignment 	
		//transform_cloud_(A, pcl_ptr_, pclTransformed_ptr_, "map"); // not efficient
		pcl::transformPointCloud(*pcl_ptr_, *pclTransformed_ptr_, A);
		
		pclTransformed_ptr_->header.frame_id = "map";	
		pcl::toROSMsg(*pclTransformed_ptr_, output);
		pc_pub_.publish(output);
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

// alright, this transform_cloud is inefficient
void Perception_Model::transform_cloud_(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_ptr, string frame) 
{
	output_cloud_ptr->header.frame_id = frame;
    output_cloud_ptr->header = input_cloud_ptr->header;
    output_cloud_ptr->is_dense = input_cloud_ptr->is_dense;
    output_cloud_ptr->width = input_cloud_ptr->width;
    output_cloud_ptr->height = input_cloud_ptr->height; 
    int npts = input_cloud_ptr->points.size();
    output_cloud_ptr->points.resize(npts);
    
    pcl::PointXYZRGB pcl_pt;
    Eigen::Vector3f pt1, pt2;
    for (int i = 0; i < npts; ++i) 
	{
        pt1 = input_cloud_ptr->points[i].getVector3fMap();
        pt2 = A * pt1;          
        pcl_pt.x = pt2(0);
        pcl_pt.y = pt2(1);
        pcl_pt.z = pt2(2);            
        pcl_pt.rgb = input_cloud_ptr->points[i].rgb;
        output_cloud_ptr->points[i] = pcl_pt;
    }
    
}
