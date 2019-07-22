/// mavros commander
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

#ifndef Mavros_Commander_H_
#define Mavros_Commander_H_

#include <iostream> // for C++
#include <fstream>
#include <string>
#include <math.h>

#include <ros/ros.h> // for ROS
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPush.h>

using namespace std;

class Mavros_Commander
{
public:
	Mavros_Commander(ros::NodeHandle *nh);
	void set_arm(bool arm, int timeout);
	void set_mode(string mode, int timeout);
	void clear_wps(int timeout);
	void send_wps(int timeout);

protected:
	ros::NodeHandle nh_;
	ros::Subscriber state_sub_;
	ros::ServiceClient get_param_client_;
	ros::ServiceClient set_arming_client_;
	ros::ServiceClient set_mode_client_;
	ros::ServiceClient wp_clear_client_;
	ros::ServiceClient wp_push_client_;

	mavros_msgs::State current_state_;

	void stateCallback_(const mavros_msgs::State::ConstPtr& msg);
	

};


#endif
