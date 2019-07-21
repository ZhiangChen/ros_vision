#!/usr/bin/env python
"""
Zhiang Chen
July 2019
"""

import rospy
import actionlib
import terrain_following.msg
import numpy as np


def terrain_lookup_client(x, y):
    client = actionlib.SimpleActionClient("terrain_lookup", terrain_following.msg.terrainAction)

    #print("waiting for action server...")
    client.wait_for_server()

    goal = terrain_following.msg.terrainGoal()
    goal.x, goal.y = x, y
    client.send_goal(goal)
    #client.wait_for_result()




if __name__ == '__main__':
    rospy.init_node('terrain_lookup_client', anonymous=True)
    goal = [0.5, 0.6]
    terrain_lookup_client(goal[0], goal[1])