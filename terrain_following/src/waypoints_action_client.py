#!/usr/bin/env python
"""
Zhiang Chen
April 2019
"""

import rospy
import actionlib
import terrain_following.msg
import numpy as np

def waypoints_client():
    client = actionlib.SimpleActionClient("waypoints_mission", terrain_following.msg.waypointsAction)

    print("waiting for action server...")
    client.wait_for_server()

    waypoints = np.array(((0, 0, 4), (20, 20, 4), (20, -20, 4), (-20, -20, 4), (0, 0, 4)))

    waypoints = waypoints.astype(float).tostring()
    goal = terrain_following.msg.waypointsGoal(waypoints)
    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()




if __name__ == '__main__':
    rospy.init_node('waypoints_client', anonymous=True)
    result = waypoints_client()
    print(result)



