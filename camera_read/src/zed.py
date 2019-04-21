#!/usr/bin/env python
"""
Zhiang Chen, April 2019
"""

import rospy
import cv2
import open3d
import pcl
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import struct
import ctypes









class Camera(object):
    def __init__(self, skip_nan = False, threshold = 0.005,  read_transform = False, save_transform = True,):
        """
        :param skip_nan: set this to True when initializing ICP, then it excludes NaN in pointcloud
        :param threshold: larger threshold means having more points to be evaluated
        :param read_transform:
        :param save_transform:
        """
        rospy.Subscriber("/zed1/point_cloud/cloud_registered", PointCloud2, self.zed1_callback)
        rospy.Subscriber("/zed2/point_cloud/cloud_registered", PointCloud2, self.zed2_callback)

        self.skip_nan = skip_nan
        self.threshold = threshold
        self.save_transform = save_transform
        self.read_transform = read_transform
        self.pub = rospy.Publisher("/merged_point_cloud", PointCloud2, queue_size=1)
        self.pub1 = rospy.Publisher("/zed1_pointcloud", PointCloud2, queue_size=1)
        self.pub2 = rospy.Publisher("/zed2_pointcloud", PointCloud2, queue_size=1)
        self.zed1_xyz = None
        self.zed2_xyz = None
        self.zed1_rgb = None
        self.zed2_rgb = None
        self.transform = None
        self.zed1_arv_time = None
        self.zed2_arv_time = None
        print("ROS Node Initialized: zed_merger")

    def zed1_callback(self, data):
        self.zed1_xyz, self.zed1_rgb = self.pc2_to_xyzrgb(data)
        self.zed1_arv_time = data.header.stamp.to_time()


        #msg = xyzrgb_array_to_pointcloud2(self.zed1_xyz, self.zed1_rgb, data.header.stamp, data.header.frame_id, data.header.seq)
        #self.pub.publish(msg)
        #self.pub2.publish(data)


        if self.transform is None:
            while self.zed2_xyz is None:
                rospy.sleep(0.1)
            if (self.zed2_arv_time - self.zed1_arv_time) < 1.5:
                if not self.read_transform:
                    print("Initializing ICP")
                    self.transform = self.icp(self.zed2_xyz.copy(), self.zed1_xyz.copy())
                    if self.save_transform:
                        np.save("transform.npy", self.transform)
                else:
                    print("Reading transform")
                    self.transform = np.load("transform.npy")
                    print("Transform: ", self.transform)

        tf_pc_zed2 = open3d.PointCloud()
        tf_pc_zed2.points = open3d.Vector3dVector(self.zed2_xyz)
        tf_pc_zed2.transform(self.transform)
        tf_zed2_points = np.asarray(tf_pc_zed2.points)


        merged_xyz = np.concatenate((self.zed1_xyz, tf_zed2_points), 0)
        merged_rgb = np.concatenate((self.zed1_rgb, self.zed2_rgb), 0)


        msg = self.xyzrgb_array_to_pointcloud2(merged_xyz, merged_rgb, data.header.stamp,
                                               data.header.frame_id, data.header.seq)


        msg2 = self.xyzrgb_array_to_pointcloud2(self.zed2_xyz, self.zed2_rgb, data.header.stamp,
                                               data.header.frame_id, data.header.seq)
        self.pub.publish(msg)
        self.pub1.publish(data)
        self.pub2.publish(msg2)





    def zed2_callback(self, data):
        self.zed2_xyz, self.zed2_rgb = self.pc2_to_xyzrgb(data)
        self.zed2_arv_time = data.header.stamp.to_time()


    def icp(self, source, target):
        source = source.astype(np.float32)
        target = target.astype(np.float32)
        threshold = self.threshold

        trans_init = np.asarray(
            [[1.0, 0.0, 0.0, 0.2],
             [0.0, 1.0, 0.0, 0.1],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]])

        pc_source = open3d.PointCloud()
        pc_source.points = open3d.Vector3dVector(source)

        pc_target = open3d.PointCloud()
        pc_target.points = open3d.Vector3dVector(target)

        evaluation = open3d.evaluate_registration(pc_source, pc_target, threshold, trans_init)
        print(evaluation)
        print("Apply point-to-point ICP")
        reg_p2p = open3d.registration_icp(pc_source, pc_target, threshold, trans_init,
                                          open3d.TransformationEstimationPointToPoint())
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)

        pc_source.transform(reg_p2p.transformation)
        source = np.asarray(pc_source.points)

        return reg_p2p.transformation

    def pc2_to_xyzrgb(self, ros_cloud):
        xyz_list = []
        rgb_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=self.skip_nan):
            xyz_list.append([data[0], data[1], data[2]])
            rgb_list.append(data[3])

        return np.asarray(xyz_list), np.asarray(rgb_list).reshape(-1, 1)

    def xyzrgb_array_to_pointcloud2(self, points, colors, stamp=None, frame_id=None, seq=None):
        '''
        Create a sensor_msgs.PointCloud2 from an array
        of points.
        '''
        msg = PointCloud2()
        assert(points.shape[0] == colors.shape[0])

        if stamp:
            msg.header.stamp = stamp
        if frame_id:
            msg.header.frame_id = frame_id
        if seq:
            msg.header.seq = seq

        N = points.shape[0]
        msg.height = 1
        msg.width = N
        xyzrgb = np.concatenate((points, colors), axis=1).astype(np.float32)


        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * N
        msg.is_dense = False
        msg.data = xyzrgb.tostring()

        return msg


if __name__ == "__main__":
    rospy.init_node("zed_merger", anonymous=False)
    zed_merger = Camera(skip_nan=False, threshold=0.5, read_transform=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")