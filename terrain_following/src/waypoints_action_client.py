#!/usr/bin/env python
"""
Zhiang Chen
May 2019
"""

import rospy
import actionlib
import terrain_following.msg
import numpy as np
import math

def waypoints_client(waypoints):
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
    rospy.init_node('waypoints_client', anonymous=True)

    #waypoints = np.array(((0, 0, 3), (2, 0, 3), (4, 0, 3), (8, 0, 3), (10, 0, 3), (8, 0, 3), (6, 0, 3), (4, 0, 3), (2, 0, 3), (0, 0, 3)))
    waypoints = np.array(((0, 0, 3), (8, 0, 3), (0, 0, 3)))
    #waypoints = np.array(((0, 0, 3)))
    #waypoints = np.array([(0, x, 4 + math.sin(x)) for x in np.arange(0, 20, 0.1)])
    result = waypoints_client(waypoints.copy())
    print(result)



