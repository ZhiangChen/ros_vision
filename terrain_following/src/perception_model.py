#!/usr/bin/env python2
"""
Zhiang Chen
April 2020
"""

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import tf
import message_filters
import actionlib
import terrain_following.msg
import open3d as o3d
import copy
import time
import math
from sklearn.neighbors import KDTree
from sklearn import linear_model

Preset_TF = True

class Perception_Model(object):
    def __init__(self):
        self.tfROS = tf.TransformerROS()
        self.pcd = o3d.geometry.PointCloud()
        self.tf_xyz = np.array(None)
        self.state_update = False
        self.ransac = linear_model.RANSACRegressor()

        if not Preset_TF:
            rospy.loginfo("checking tf from camera to base_link ...")
            listener = tf.TransformListener()
            while not rospy.is_shutdown():
                try:
                    now = rospy.Time.now()
                    listener.waitForTransform("base_link", "r200", now, rospy.Duration(5.0))
                    (trans, rot) = listener.lookupTransform("base_link", "r200", now)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
        else:
            trans_vec = (0.1, 0, - 0.01)
            trans = tf.transformations.translation_matrix(trans_vec)
            quaternion = (-0.6743797, 0.6743797, - 0.2126311, 0.2126311)
            rot = tf.transformations.quaternion_matrix(quaternion)

        self.T_camera2base = np.matmul(trans, rot)

        self.sub_pc = message_filters.Subscriber("/r200/depth/points", PointCloud2)
        self.sub_pose = message_filters.Subscriber('/mavros/local_position/pose', PoseStamped)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_pc, self.sub_pose], queue_size=100, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.pc_pub = rospy.Publisher("/world_point_cloud", PointCloud2, queue_size=1)
        self.pp_pub = rospy.Publisher("/path_points", PointCloud2, queue_size=1)
        self.pose_pub = rospy.Publisher("/posestamped", PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher("/terrain_following_path", Path, queue_size=1)


        self._result = terrain_following.msg.terrainResult()
        self._as = actionlib.SimpleActionServer("terrain_lookup", terrain_following.msg.terrainAction,
                                                execute_cb=self.exCallback, auto_start=False)
        self._as.start()
        print("perception_model initialized!")

    def callback(self, pc_msg, pose_msg):
        """
        1. get pointcloud;
        2. get T_base2world, T_camera2world
        3. get transformed pointcloud
        """
        self.q = pose_msg.pose.orientation
        self.pos = pose_msg.pose.position
        self.T_base2world = self.tfROS.fromTranslationRotation((self.pos.x, self.pos.y, self.pos.z), (self.q.x, self.q.y, self.q.z, self.q.w))
        self.T_camera2world = np.matmul(self.T_base2world, self.T_camera2base,)

        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg)
        points = np.insert(xyz, 3, 1, axis=1)
        tf_xyz = np.matmul(points, self.T_camera2world.transpose())[:, :3]
        self.tf_xyz = copy.deepcopy(tf_xyz)
        #tf_pc_msg = self.xyz_array_to_pointcloud2(tf_xyz, pc_msg.header.stamp, frame_id="map")

        #self.pc_pub.publish(tf_pc_msg)
        self.state_update = True


    def exCallback(self, goal):
        path_msg = Path()

        if self.state_update:
            self.state_update = False
            t1 = time.time()
            # get desire position and current position
            xyz_des = np.asarray((goal.x_des, goal.y_des, self.pos.z))
            xyz_cur = np.asarray((self.pos.x, self.pos.y, self.pos.z))
            # get transformation (or orientation and translation) of a global path
            dir_vec = xyz_des - xyz_cur

            #  bug: angle is not correct
            angle = math.atan2(dir_vec[1], dir_vec[0]) - math.atan2(0, 1)
            print(dir_vec)
            print(angle)
            q = tf.transformations.quaternion_about_axis(angle, (0, 0, 1))
            T_box2world = self.tfROS.fromTranslationRotation((xyz_cur[0], xyz_cur[1], xyz_cur[2]),
                                                             (q[0], q[1], q[2], q[3]))

            # transform pointclouds
            points = np.insert(self.tf_xyz, 3, 1, axis=1)
            tf_xyz = np.matmul(points, np.linalg.inv(T_box2world).transpose())[:, :3]

            # filter points in box
            box_width = 0.1
            right_boundary = tf_xyz[:, 1] < (box_width / 2)
            left_boundary = tf_xyz[:, 1] > (-box_width / 2)
            front_boundary = tf_xyz[:, 0] <= np.linalg.norm(dir_vec)
            indices = np.all((left_boundary, right_boundary, front_boundary), axis=0)
            tf_xyz_box = tf_xyz[indices]

            # get 2D normal estimation of the filtered points
            tf_xz_box = np.delete(tf_xyz_box, 1, 1)
            sample_size = 200
            if tf_xz_box.shape[0] > sample_size:
                np.random.shuffle(tf_xz_box)
                sampled_xz_box = tf_xz_box[:sample_size]
            else:
                sampled_xz_box = tf_xz_box

            if sampled_xz_box.shape[0] < 30:
                self._result.z = goal.z
                self._result.local_path = path_msg
                self._result.got_terrain = False
                self._as.set_succeeded(self._result)
                return None

            normals = self.getNormals(sampled_xz_box, tf_xz_box)

            # set all normals pointing toward drone

            # bug: RANSAC might be wrong; maybe smooth first and then compute normals
            # RANSAC remove outliers: linearize gradient of local path and remove outliers
            # which means the gradient of the terrain should be continuous
            #print(normals.shape)
            self.ransac.fit(normals[:, 0].reshape(-1, 1), normals[:, 1].reshape(-1, 1))
            inlier_mask = self.ransac.inlier_mask_
            refined_normals = normals[inlier_mask]
            #print(refined_normals.shape)

            # generate points using normal vectors
            path_points_xz = sampled_xz_box[inlier_mask] + refined_normals * goal.relative_height
            path_points_xyz = np.insert(path_points_xz, 1, 0, axis=1)
            start_bounding = path_points_xyz[:, 0] >= 0
            desire_bounding = path_points_xyz[:, 0] <= np.linalg.norm(dir_vec)
            indices = np.all((start_bounding, desire_bounding), axis=0)
            path_points_xyz = path_points_xyz[indices]


            # fit a curve using the generated points
            # transform the curve back



            # self.pcd.points = o3d.utility.Vector3dVector(self.tf_xyz)
            # self.pcd.points = o3d.utility.Vector3dVector(self.tf_xyz)

            tf_pc_msg = self.xyz_array_to_pointcloud2(tf_xyz_box, frame_id="map")
            tf_path_points_msg = self.xyz_array_to_pointcloud2(path_points_xyz, frame_id="map")
            #print(tf_xyz_box.shape)
            self.pc_pub.publish(tf_pc_msg)
            self.pp_pub.publish(tf_path_points_msg)
            #print(time.time() - t1)

            self._result.z = 5
            self._result.local_path = path_msg
            self._result.got_terrain = True
            self._as.set_succeeded(self._result)

        else:
            self._result.z = goal.z
            self._result.local_path = path_msg
            self._result.got_terrain = False
            self._as.set_succeeded(self._result)





    def getNormals(self, query_points, points):
        """
        get normals of query_points in points
        :param points:
        :return:
        """
        tree = KDTree(points, leaf_size=2)
        queries = [tree.query(point.reshape(1, -1), k=30) for point in query_points]
        normals = np.asarray([self.normalEstimation(dist, points[indices]) for (dist, indices) in queries])
        return normals

    def normalEstimation(self, distances, points):
        """
        estimate the normal vector of points
        :param distances:
        :param points:
        :return:
        """
        points = points.reshape(-1, 2)
        points_vec = (points - points[0, :])
        # to find a normal vector, we need to solve a overdetermined homogeneous system:
        # np.matmul(points_vec, N) = 0
        # then N is the normal vector
        A = np.matmul(points_vec.transpose(), points_vec)
        w, v = np.linalg.eig(A)
        i = np.argmin(np.absolute(w))
        N = v[:, i]
        return N

    def xyz_array_to_pointcloud2(self, points, stamp=None, frame_id=None):
        msg = PointCloud2()
        if stamp is not None:
            msg.header.stamp = stamp
        if frame_id is not None:
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


if __name__ == "__main__":
    rospy.init_node("perception_model", anonymous=False)
    pm = Perception_Model()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")
