/// mavros_vel_controller_cpp.cpp
/// Zhiang Chen, Aug 2019
/// MIT License

#include <terrain_following/waypoints_vel_controller.h>

Waypoints_Vel_Controller::Waypoints_Vel_Controller(ros::NodeHandle *nh): nh_(*nh), mavros_commander_(new Mavros_Commander(nh)),
waypoints_server_(*nh, "waypoints_mission", boost::bind(&Waypoints_Vel_Controller::waypointsCallback_, this, _1), false),
terrain_client_("terrain_lookup", true)
{
	vel_setpoint_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1, true); // for debug

	ros::Duration timeout = ros::Duration(5);
	try
	{
		ros::service::waitForService("terrain_lookup", timeout);
	}
	catch (...)
	{
		ROS_ERROR("Failed to connect Perception_Model service.");
	}
	ROS_INFO("Perception_Model service connected.");

	waypoints_server_.start();
	ROS_INFO("waypoints_server started.");

	ROS_INFO("Waypoints_Vel_Controller initialized.");

}


void Waypoints_Vel_Controller::waypointsCallback_(const terrain_following::waypointsGoal::ConstPtr &goal)
{
	// this needs to be re-visited
}
