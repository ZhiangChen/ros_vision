/// perception_model_cpp_main.cpp
/// Zhiang Chen, July 2019
/// MIT License

#include <terrain_following/perception_model_cpp.h>
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "perception_model");
	ros::NodeHandle nh;
	//Perception_Model PM(&nh, "/r200/depth/points"); 
	Perception_Model PM(&nh); // message filter approximate synchronization 
	
	//ros::spin(); // single thread
	
	//ros::MultiThreadedSpinner spinner(4); // multi-thread
	//spinner.spin();

	ros::AsyncSpinner spinner(4); // asynchronized multi-thread
	spinner.start();
	ros::waitForShutdown();
	
	/*
	while(ros::ok())
	{
		ros::Duration(1).sleep();
	}
	*/

	return 0;
}
