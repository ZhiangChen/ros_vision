#!/usr/bin/env python
"""
Zhiang Chen
Aug 2019
"""

import rospy
import actionlib
import terrain_following.msg
import numpy as np
import math

def ros_record(topics):
    client = actionlib.SimpleActionClient("waypoints_mission", terrain_following.msg.waypointsAction)

    #print("waiting for action server...")
    client.wait_for_server()
    waypoints = waypoints.astype(float).tostring()
    goal = terrain_following.msg.waypointsGoal(waypoints)
    client.send_goal(goal)
    print("goal sent")
    client.wait_for_result()

    return client.get_result()




if __name__ == '__main__':
    rospy.init_node('ros_topic_recorder', anonymous=True)
    topics = [""]
    ros_record(topics)
    
