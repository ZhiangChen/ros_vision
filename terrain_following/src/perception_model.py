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
from sklearn.neighbors import KDTree
import tf

class Depth_Camera(object):
    def __init__(self):
        rospy.Subscriber("/r200/depth/points", PointCloud2, self.pointcloud_callback)
        self.pc_pub = rospy.Publisher("/world_point_cloud", PointCloud2, queue_size=1)
        # self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)

        self.tfROS = tf.TransformerROS()
        self.listener = tf.TransformListener()

    def estimate_terrain_vertical_distance(self, pinpoint, k=3):
        '''
        estimate the vertical of terrain at pintpoint
        :param pinpoint: (x,y) w.r.t global coord
        :return: vertical distance
        '''
        pinpoint = np.array(pinpoint).reshape(1, -1)
        tree = KDTree(self.w_points[:, :2], leaf_size=2)
        dist, ind = tree.query(pinpoint, k=k)
        terrain_points = self.w_points[ind].reshape((-1, 3))

        return terrain_points.mean(axis=0)[-1]

    
    def pointcloud_callback(self, ros_cloud):
        self.ros_cloud = ros_cloud


        xyz_list = []
        for data in pc2.read_points(ros_cloud, skip_nans=True):
            xyz_list.append([data[0], data[1], data[2]])    # this is so bad, inefficient
        self.points = np.array(xyz_list).astype(float)

        #self.pc_pub.publish(ros_cloud)
        if self.points.shape[0] != 0:
            try:
                (trans, rot) = self.listener.lookupTransform('r200', 'map', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                None

            trans = (trans[1], trans[0], trans[2])  # bug: x,y are swapped

            T = self.tfROS.fromTranslationRotation(trans, rot)

            points = np.insert(self.points, 3, 1, axis=1)
            self.w_points = np.matmul(points, T.transpose())[:, :3]
            msg = self.xyz_array_to_pointcloud2(self.w_points, "map")
            self.pc_pub.publish(msg)





    def xyz_array_to_pointcloud2(self, points, frame_id):
        msg = PointCloud2()
        msg.header.stamp = self.ros_cloud.header.stamp
        msg.header.frame_id = frame_id
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


    def local_position_callback(self, data):
        self.local_position = data


if __name__ == "__main__":
    rospy.init_node("depth_read", anonymous=False)
    realsense = Depth_Camera()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")
