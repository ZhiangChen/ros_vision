#include <terrain_following/mavros_commander.h>

using namespace std;

Mavros_Commander::Mavros_Commander(ros::NodeHandle *nh): nh_(*nh)
{
	
	ros::Duration timeout = ros::Duration(5);
	try
	{
		ros::service::waitForService("mavros/param/get", timeout);
		ros::service::waitForService("mavros/cmd/arming", timeout);
		ros::service::waitForService("mavros/set_mode", timeout);
		ros::service::waitForService("mavros/mission/clear", timeout);
		ros::service::waitForService("mavros/mission/push", timeout);
	}
	catch (...)
	{
		ROS_ERROR("Failed to connect services.");
	}
	ROS_INFO("Serviced connected.");	
	

	get_param_client_ = nh_.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
	set_arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	wp_clear_client_ = nh_.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
	wp_push_client_ = nh_.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &Mavros_Commander::stateCallback_, this);

	ros::Rate rate(20.0); // wait for FCU connection
	current_state_.connected = false;
    while(ros::ok() && !current_state_.connected)
	{
        ros::spinOnce();
        rate.sleep();
    }
	ROS_INFO("FCU connected.");
	ROS_INFO("Mavros_Commander initialized.");
}

void Mavros_Commander::set_mode(string mode, int timeout)
{
	ROS_INFO_STREAM("Setting FCU mode: " << mode);
	ros::Rate rate(1);
	bool mode_set = false;
	for (int i = 0; i<timeout; i++)
	{
		if (current_state_.mode == mode)
		{
			ROS_INFO("Set mode succeeded.");
			break;
		}
		else
		{
			try
			{
				mavros_msgs::SetMode setmode_srv;
				setmode_srv.request.custom_mode = mode;
				if (set_mode_client_.call(setmode_srv) && setmode_srv.response.mode_sent)
				{
					ROS_INFO("Set mode succeeded.");
					break;
				}
			}
			catch (...)
			{
				ROS_ERROR("Set mode failed.");
			}
		}
		rate.sleep();
	}
}

void Mavros_Commander::set_arm(bool arm, int timeout)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	ros::Rate rate(1);
	bool mode_set = false;
	for (int i = 0; i<timeout; i++)
	{
		if (arm)
		{
			ROS_INFO("Setting FCU mode: armed");
			if (!current_state_.armed)
			{
				try
				{
					if (set_arming_client_.call(arm_cmd) && arm_cmd.response.success)
					{
						ROS_INFO("Vehicle armed.");
						break;
					}
				}
				catch (...)
				{
					ROS_ERROR("Vehicle arm failed.");
				}
			}
			else
			{
				ROS_INFO("Vehicle armed.");
				break;
			}
		}
		else
		{
			ROS_INFO("Setting FCU mode: disarmed");
			if (!current_state_.armed)
			{
				try
				{
					if (set_arming_client_.call(arm_cmd) && arm_cmd.response.success)
					{
						ROS_INFO("Vehicle disarmed.");
						break;
					}
				}
				catch (...)
				{
					ROS_ERROR("Vehicle disarm failed.");
				}
			}
			else
			{
				ROS_INFO("Vehicle disarmed.");
				break;
			}
		}
		rate.sleep();
	}
    
}

void Mavros_Commander::stateCallback_(const mavros_msgs::State::ConstPtr& msg)
{
	current_state_ = *msg;
}
