/// Perception model for terrain following
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

#ifndef Perception_Model_H_
#define Perception_Model_H_

#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h> 
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
//#include <pcl-1.7/pcl/point_cloud.h>
//#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

class Perception_Model
{
public:
	Perception_Model(ros::NodeHandle *nh, string pc_topic);
	//Perception_Model(string pc_topic);

private:
	ros::NodeHandle nh_;
	string pc_topic_;

	ros::Subscriber pc_sub_;
	ros::Publisher pc_pub_;

	void pcCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
};

#endif
