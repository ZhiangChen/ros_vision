/// waypoints_vel_controller.h
/// Zhiang Chen, Aug 2019
/// MIT License

#ifndef WAYPOINTS_VEL_CONTROLLER_H_
#define WAYPOINTS_VEL_CONTROLLER_H_

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

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <terrain_following/mavros_commander.h> // terrain following
#include <terrain_following/waypointsAction.h>
#include <terrain_following/terrainAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>


using namespace std;

class Waypoints_Vel_Controller
{
public:
    Waypoints_Vel_Controller(ros::NodeHandle *nh);

protected:
    ros::NodeHandle nh_;
    Mavros_Commander* mavros_commander_;
    actionlib::SimpleActionServer<terrain_following::waypointsAction> waypoints_server_;
    actionlib::SimpleActionClient<terrain_following::terrainAction> terrain_client_;
    ros::Publisher vel_setpoint_pub_;

    void waypointsCallback_(const terrain_following::waypointsGoal::ConstPtr &goal);


};

#endif
