#include <terrain_following/perception_model_cpp.h>
#include <ros/ros.h>
#include <string>

using namespace std;


Perception_Model::Perception_Model(ros::NodeHandle *nh): nh_(*nh),
pcl_rgbd_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>),
pcl_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>),
pclTransformed_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>),
pclTransformed_ptr_copy_(new pcl::PointCloud<pcl::PointXYZRGB>),
pclTransformed_xyz_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
boxTransformed_xyz_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
boxTransformed_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>),
as_(*nh, "terrain_lookup", boost::bind(&Perception_Model::executeCB, this, _1), false)
{
    cout << PCL_VERSION << endl;
	pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/world_point_cloud", 1, true);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/posestamped", 1, true); // for debug
	path_pub_ = nh_.advertise<nav_msgs::Path>("/terrain_following_path", 1, true);

	mf_pc_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/r200/depth/points", Queue_Size);
	mf_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "mavros/local_position/pose", Queue_Size);
	pose_pc_sync_ = new message_filters::Synchronizer<PC_POSE_POLICY>(PC_POSE_POLICY(Queue_Size), *mf_pc_sub_, *mf_pose_sub_);
	pose_pc_sync_->registerCallback(boost::bind(&Perception_Model::mfCallback, this, _1, _2));

    pclTransformed_ptr_copy_ -> is_dense = false;
    pcl_ptr_ -> is_dense = false;

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
    body_pose_ = *pose;
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pc.is_dense = false;
	pcl::fromROSMsg(*cloud, pc);

	//pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    //sor.setInputCloud(pcl_ptr);
    //sor.setLeafSize(0.01f, 0.01f, 0.01f);
    //sor.filter(*pcl_ptr_);
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(pc, *pcl_ptr_, idx);

	T_base2world_ = Posestamped2Affine3d_(body_pose_);
	T_camera2world_ = T_base2world_ * T_camera2base_;
	Eigen::Affine3f A = T_camera2world_.cast<float>(); // need to convert to Eigen::Affine3f, which is compatible with pointcloud assignment

	pcl::transformPointCloud(*pcl_ptr_, *pclTransformed_ptr_, A);
	pclTransformed_ptr_->header.frame_id = "map";


	pcl::copyPointCloud(*pclTransformed_ptr_, *pclTransformed_ptr_copy_);

	camera_pose_.pose = Affine3d2Pose_(T_camera2world_); // for debug
	pose_pub_.publish(pose); // for debug
	//sensor_msgs::PointCloud2 output; // for debug
	//pcl::toROSMsg(*pclTransformed_ptr_, output); // for debug
	//pc_pub_.publish(output); // for debug

	//t = ros::Time::now().toSec();
	//cout << t << endl;

	g_path_.poses.push_back(body_pose_);
	path_pub_.publish(g_path_);
	pc_got_ = true;
}


void Perception_Model::executeCB(const terrain_following::terrainGoalConstPtr &goal)
{

	double x = goal->x;
	double y = goal->y;
	double z = goal->z;
	double relative_height = goal->relative_height;
    double x_des = goal->x_des;
    double y_des = goal->y_des;

	if (pc_got_)
	{
	    pc_got_ = false;
	    getLocalPath(x_des, y_des);
		pcl::CropBox<pcl::PointXYZRGB> box_crop;
		box_crop.setMin(Eigen::Vector4f(x - BOX_X/2.0, y - BOX_Y/2.0, -10000, 1.0));
		box_crop.setMax(Eigen::Vector4f(x + BOX_X/2.0, y + BOX_Y/2.0, 10000, 1.0));
		box_crop.setTranslation(no_translation_);
		box_crop.setRotation(no_rotation_);
		//Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		//box_crop.setTransform(transform); // NEVER USE THIS! It doesn't work!
		box_crop.setInputCloud(pclTransformed_ptr_copy_);
		box_crop.filter(*boxTransformed_ptr_);

		if (boxTransformed_ptr_->size() > BOX_POINTS_THRES)
		{
            //Eigen::Vector4f box_centroid_pcl;
			//cout << boxTransformed_ptr_->size() <<endl;
			pcl::compute3DCentroid(*boxTransformed_ptr_, box_centroid_pcl_);
			//cout << box_centroid_pcl_[0] <<" " << box_centroid_pcl_[1] << " " << box_centroid_pcl_[2]<<endl;

			sensor_msgs::PointCloud2 output; // for debug
			pcl::toROSMsg(*boxTransformed_ptr_, output); // for debug
			pc_pub_.publish(output); // for debug

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

nav_msgs::Path Perception_Model::getLocalPath(double x_des, double y_des)
{
    nav_msgs::Path local_path;
    /*********************************************
    get point indices along terrain to destination
    **********************************************/
    // get geometry
    double x = body_pose_.pose.position.x;
    double y = body_pose_.pose.position.y;
    Eigen::Vector2d current_xy(x, y);
    Eigen::Vector2d desired_xy(x_des, y_des);
    Eigen::Vector2d dir_v(x_des - x, y_des - y);
    double box_length = dir_v.norm();
    double box_width = 0.1;
    tf::Vector3 dir_vec(x_des - x, y_des - y, 0);
    dir_vec = dir_vec.normalize();
    tf::Vector3 x_vec(1, 0, 0);
    double angle = dir_vec.angle(x_vec);

    double dot = dir_vec[0]*1;     // dot product between [x1, y1] and [x2, y2]
    double det = - dir_vec[1]*1;       // determinant
    angle = -atan2(det, dot);  // atan2(y, x) or atan2(sin, cos)

    // define box
    pcl::CropBox<pcl::PointXYZRGB> boxCrop;
    boxCrop.setMin(Eigen::Vector4f(0, -box_width/2.0, -10000, 1.0));
    boxCrop.setMax(Eigen::Vector4f(box_length, +box_width/2.0, 10000, 1.0));
    //Eigen::Affine3f A = T_box2world.cast<float>();
    //boxCrop.setTransform(A); // This is terrible!
    Eigen::Vector3f translation(body_pose_.pose.position.x, body_pose_.pose.position.y, 0);
	Eigen::Vector3f rotation(0, 0, angle);
    boxCrop.setTranslation(translation);
    boxCrop.setRotation(rotation);
    boxCrop.setInputCloud(pclTransformed_ptr_copy_);

    // get point indices in box
    vector<int> indices_inside;
    boxCrop.filter(indices_inside);
    cout << "idx: " << indices_inside.size() << endl;



    // for debug
    if (indices_inside.size() > 10)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr boxTransformed_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        boxCrop.filter(*boxTransformed_ptr);

        sensor_msgs::PointCloud2 output; // for debug
        pcl::toROSMsg(*boxTransformed_ptr, output); // for debug
        pc_pub_.publish(output); // for debug
    }
    //*/

    /*********************************************
    get normal estimation of the indices
    **********************************************/
    // 3D normal estimation is wrong
    /*
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(pclTransformed_ptr_copy_);


    // Pass the indices
    boost::shared_ptr<std::vector<int>> indicesptr (new std::vector<int> (indices_inside));
    ne.setIndices (indicesptr);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm

    ne.setRadiusSearch (0.3);

    // Compute the features
    ne.compute(*cloud_normals); // This API has a bug. ROS is using pcl 1.8.1
    // the bug is fixed in pcl >= 1.8.3
    */

    /*
    for(int i = 0; i < cloud_normals->points.size(); ++i)
        cout << "(" << cloud_normals->at(i).normal[0]
        << ", "    << cloud_normals->at(i).normal[1]
         << ", "    << cloud_normals->at(i).normal[2] << ")";
    */
    //cout<<"normal: "<< cloud_normals->points.size()<<endl;

    /*********************************************
    generate path points
    **********************************************/

    /*********************************************
    smooth path points
    **********************************************/


    return local_path;
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



