#!/usr/bin/env python2
"""
Zhiang Chen
May 2019
"""

import rospy
import cv2
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import struct
import ctypes
from sklearn.neighbors import KDTree
import tf

class Depth_Camera(object):
    def __init__(self):
        rospy.Subscriber("/r200/depth/points", PointCloud2, self.pointcloud_callback)


    def estimate_terrain_vertical_distance(self, pinpoint):
        """
        estimate the vertical of terrain at pintpoint
        :param pinpoint: (x,y) w.r.t global coord
        :return: vertical distance
        """
        tree = KDTree(self.points[:,:2], leaf_size=2)
        dist, ind = tree.query(pinpoint, k=3)
        terrain_points = self.points[ind]


    def pointcloud_callback(self, ros_cloud):
        xyz_list = []
        for data in pc2.read_points(ros_cloud, skip_nans=True):
            xyz_list.append([data[0], data[1], data[2]])
        self.points = np.array(xyz_list).astype(float)






if __name__ == "__main__":
    rospy.init_node("depth_read", anonymous=False)
    realsense = Depth_Camera()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")
