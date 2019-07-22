/// mavros_commander.cpp
/// Zhiang Chen, July 2019
/// MIT License

#include <terrain_following/mavros_commander.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mavros_commander");
	ros::NodeHandle nh;
	Mavros_Commander mc(&nh); // message filter approximate synchronization 
	
	//ros::spin(); // single thread
	
	//ros::MultiThreadedSpinner spinner(4); // multi-thread
	//spinner.spin();

	//ros::AsyncSpinner spinner(4); // asynchronized multi-thread
	//spinner.start();
	//ros::waitForShutdown();
	
	ros::spinOnce();
	mc.set_mode("OFFBOARD", 5);
	mc.set_arm(true, 5);
	
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
	geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
	
	while(ros::ok())
	{
	local_pos_pub.publish(pose);
	ros::spinOnce();
	ros::Duration(.02).sleep();
	}
	//mc.set_mode("ATUO.LAND", 5);
	//mc.set_arm(false, 5);
	return 0;
}

