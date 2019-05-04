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
        self.pub = rospy.Publisher("/world_point_cloud", PointCloud2, queue_size=1)

        self.listener = tf.TransformListener()
        self.tfROS = tf.TransformerROS()



    def estimate_terrain_vertical_distance(self, pinpoint):
        """
        estimate the vertical of terrain at pintpoint
        :param pinpoint: (x,y) w.r.t global coord
        :return: vertical distance
        """
        try:
            (trans, rot) = self.listener.lookupTransform('r200', 'local_origin', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            None

        trans = (trans[1], trans[0], trans[2]) # bug: x,y are swapped
        T = self.tfROS.fromTranslationRotation(trans, rot)
        points = np.insert(self.points, 3, 1, axis=1)
        w_points = np.matmul(points, T.transpose())[:,:3]

        #msg = self.xyz_array_to_pointcloud2(w_points)
        #self.pub.publish(msg)

        pinpoint = np.array(pinpoint).reshape(1, -1)
        tree = KDTree(w_points[:, :2], leaf_size=2)
        dist, ind = tree.query(pinpoint, k=3)
        terrain_points = w_points[ind]

        print(terrain_points)

    def pointcloud_callback(self, ros_cloud):
        self.ros_cloud = ros_cloud

        xyz_list = []
        for data in pc2.read_points(ros_cloud, skip_nans=True):
            xyz_list.append([data[0], data[1], data[2]])
        self.points = np.array(xyz_list).astype(float)
        self.estimate_terrain_vertical_distance((.2, .2))

    def xyz_array_to_pointcloud2(self, points):
        msg = PointCloud2()
        msg.header.stamp = self.ros_cloud.header.stamp
        msg.header.frame_id = "local_origin"

        N = points.shape[0]
        msg.height = 1
        msg.width = N
        xyz = points.astype(np.float32)

        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * N
        msg.is_dense = False
        msg.data = xyz.tostring()

        return msg



if __name__ == "__main__":
    rospy.init_node("depth_read", anonymous=False)
    realsense = Depth_Camera()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")
