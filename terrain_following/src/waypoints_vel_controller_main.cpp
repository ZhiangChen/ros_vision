/// waypoints_vel_controller_main.cpp
/// Zhiang Chen, Aug 2019
/// MIT License

#include <terrain_following/waypoints_vel_controller.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoints_vel_controller_main");
	ros::NodeHandle nh;
	Waypoints_Vel_Controller vel_controller(&nh);

    return 0;
}
