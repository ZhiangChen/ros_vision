#!/usr/bin/env python2
"""
Zhiang Chen
April 2020
"""

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
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
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image

Preset_TF = True

class Perception_Model(object):
    def __init__(self):
        self.tfROS = tf.TransformerROS()
        self.pcd = o3d.geometry.PointCloud()
        self.tf_xyz = np.array(None)
        self.state_update = False
        self.ransac = linear_model.RANSACRegressor()
        self.bridge = CvBridge()
        self.pub_xy_terrain = rospy.Publisher("/perception_model/xz_terrain", Image, queue_size=1)

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
            quaternion = (-0.67437972,  0.67437972, -0.21263111,  0.21263111)
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
        self.tpath_pub = rospy.Publisher("/terrain_path", Path, queue_size=1)
        self.tpoints_pub = rospy.Publisher("/terrain_points", PointCloud2, queue_size=1)
        self.local_path_pub = rospy.Publisher("/local_path", Path, queue_size=1)


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
        tf_pc_msg = self.xyz_array_to_pointcloud2(tf_xyz, pc_msg.header.stamp, frame_id="map")
        self.pc_pub.publish(tf_pc_msg)
        self.pose_pub.publish(pose_msg)
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
            # voxelize points
            self.pcd.points = o3d.utility.Vector3dVector(tf_xyz_box)
            downpcd = self.pcd.voxel_down_sample(voxel_size=0.05)
            tf_xyz_box = np.asarray(downpcd.points)
            print("Box voxel #: " + str(tf_xyz_box.shape[0]))


            # get 2D points
            tf_xz_box = np.delete(tf_xyz_box, 1, 1)
            sample_size = 100
            if tf_xz_box.shape[0] > sample_size:
                np.random.shuffle(tf_xz_box)
                sampled_xz_box = tf_xz_box[:sample_size]
            else:
                sampled_xz_box = tf_xz_box

            if tf_xz_box.shape[0] < 30:
                self._result.z = goal.z
                self._result.local_path = path_msg
                self._result.got_terrain = False
                self._as.set_succeeded(self._result)
                return None

            #path_points = self.generatePathPoints(tf_xz_box, goal.relative_height)
            """
            path_points = self.generatePathPoints(tf_xz_box, 2)
            path_points_xz = np.asarray([pp['p'] for pp in path_points])

            path_points_xyz = np.insert(path_points_xz, 1, 0, axis=1)
            cur_point = np.asarray((0, 0, 0))
            path_points_xyz = np.insert(path_points_xyz, 0, cur_point, axis=0)
            path = self.points_to_path(path_points_xyz)
            self.local_path_pub.publish(path)

            terrain_points_xz = np.asarray(self.terrain_points)
            terrain_points_xyz = np.insert(terrain_points_xz, 1, 0, axis=1)
            terrain_path = self.points_to_path(terrain_points_xyz)
            self.tpath_pub.publish(terrain_path)
            terrain_path_points_msg = self.xyz_array_to_pointcloud2(terrain_points_xyz, frame_id="map")
            self.tpoints_pub.publish(terrain_path_points_msg)
            """

            #tf_xz_box_grid = self.gridify(tf_xz_box, spacing=0.01)
            #terrain_grid = tf_xz_box_grid.copy()
            #mask = np.zeros((1000, 1000))  # 10m x 10m mask
            # also make sure the image size satisfy FFT performance optimization: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_transforms/py_fourier_transform/py_fourier_transform.html#performance-optimization-of-dft
            #start = (50, 50)
            #terrain_grid[:, 0] = start[0] + tf_xz_box_grid[:, 0]
            #terrain_grid[:, 1] = start[1] - tf_xz_box_grid[:, 1]
            #rows, cols = zip(*terrain_grid)
            #mask[cols, rows] = 255
            #mask = mask.astype(np.uint8)
            #terrain_img = mask.copy()
            # Low-pass filter by 2D fourier transform
            #ff_img = self.FFT_LPF(terrain_img, 50, 30) # hmmm, maybe this is not a good idea
            # ff_img = (ff_img / ff_img.max()) * 255
            #lpf_img = cv2.GaussianBlur(terrain_img, (5, 5), 0)

            # threshold filter / adaptive threshold filter

            # uniform Euclidean distance sampling: two options: 1) original xz points; 2) filtered curve


            #image_msg = self.bridge.cv2_to_imgmsg(lpf_img.astype(np.uint8), 'mono8')
            #self.pub_xy_terrain.publish(image_msg)



            # the concept of terrain frequency


            #tf_path_points_msg = self.xyz_array_to_pointcloud2(path_points_xyz, frame_id="map")
            #self.pp_pub.publish(tf_path_points_msg)

            """Using poles to generate path points is not working well"""
            # compute poles (tangent points) of expanded circles
            # the start point is current position, P1
            # the next point, P2, is the pole of the closest circle Ci wrt P1
            # remove Ci from sampled_xz_box
            # the next point, P3, is the pole of the closest circle Cj wrt P2
            # repeat until the last circle
            # bug1: when a point is in circle
            # bug2: when the closest pole is below the terrain
            #xz_cur = np.asarray((self.pos.x, self.pos.z))
            #path_points_xz = self.findPathPoints(xz_cur, sampled_xz_box, goal.relative_height)
            #path_points_xyz = np.insert(path_points_xz, 1, 0, axis=1)
            #start_bounding = path_points_xyz[:, 0] >= 0
            #desire_bounding = path_points_xyz[:, 0] <= np.linalg.norm(dir_vec)
            #indices = np.all((start_bounding, desire_bounding), axis=0)
            #path_points_xyz = path_points_xyz[indices]

            """Using normal estimation to generate path points is not working well"""
            """Using Normal vectors of terrain to directly generate terrain is wrong"""
            """You should use tangent instead"""
            # get 2D normal estimation of the filtered points
            normals = self.getNormals(sampled_xz_box, tf_xz_box)
            sampled_z_box = sampled_xz_box[:, 1]
            """
            h0 = 0.5
            w1 = h0 / (h0 - sampled_z_box)
            w2 = -sampled_z_box / (h0 - sampled_z_box)
            w1 = np.asarray((w1, w1)).transpose()
            w2 = np.asarray((w2, w2)).transpose()
            dir = w1*normals + w2*np.asarray((0, 1))
            """
            dir = 0.2 * normals + 0.8 * np.asarray((0, 1))
            path_points_xz = sampled_xz_box + dir * goal.relative_height
            path_points_xz = self.repulse(path_points_xz, tf_xz_box)
            path_points_xyz = np.insert(path_points_xz, 1, 0, axis=1)

            local_path = path_points_xz.copy()
            z = np.polyfit(local_path[:, 0], local_path[:, 1], 7)
            p = np.poly1d(z)
            local_path[:, 0] = np.sort(local_path[:, 0])
            local_path[:, 1] = p(local_path[:, 0])

            """
            # curve fitting in polar coordinates
            path_points_xz_polar = np.asarray([self.cart2pol(x, y) for x, y in path_points_xz])
            z = np.polyfit(path_points_xz_polar[:, 0], path_points_xz_polar[:, 1], 7)
            p = np.poly1d(z)
            path_points_xz_polar[:, 0] = np.sort(path_points_xz_polar[:, 0])
            path_points_xz_polar[:, 1] = p(path_points_xz_polar[:, 0])
            local_path = np.asarray([self.pol2cart(rho, phi) for rho, phi in path_points_xz_polar])
            """
            local_path = np.insert(local_path, 1, 0, axis=1)

            points = np.insert(local_path, 3, 1, axis=1)
            local_path = np.matmul(points, T_box2world.transpose())[:, :3]

            points = np.insert(path_points_xyz, 3, 1, axis=1)
            path_points_xyz = np.matmul(points, T_box2world.transpose())[:, :3]

            points = np.insert(tf_xyz_box, 3, 1, axis=1)
            tf_xyz_box = np.matmul(points, T_box2world.transpose())[:, :3]

            # set all normals pointing toward drone
            # bug: RANSAC might be wrong; maybe smooth first and then compute normals
            # RANSAC remove outliers: linearize gradient of local path and remove outliers
            # which means the gradient of the terrain should be continuous
            #print(normals.shape)
            #self.ransac.fit(normals[:, 0].reshape(-1, 1), normals[:, 1].reshape(-1, 1))
            #inlier_mask = self.ransac.inlier_mask_
            #refined_normals = normals[inlier_mask]
            #print(refined_normals.shape)

            # generate points using normal vectors
            #path_points_xz = sampled_xz_box[inlier_mask] + refined_normals * goal.relative_height
            #path_points_xz = sampled_xz_box + normals * goal.relative_height
            #path_points_xyz = np.insert(path_points_xz, 1, 0, axis=1)
            #start_bounding = path_points_xyz[:, 0] >= 0
            #desire_bounding = path_points_xyz[:, 0] <= np.linalg.norm(dir_vec)
            #indices = np.all((start_bounding, desire_bounding), axis=0)
            #path_points_xyz = path_points_xyz[indices]
            # fit a curve using the generated points
            # transform the curve back
            # self.pcd.points = o3d.utility.Vector3dVector(self.tf_xyz)
            # self.pcd.points = o3d.utility.Vector3dVector(self.tf_xyz)
            #tf_pc_msg = self.xyz_array_to_pointcloud2(tf_xyz_box, frame_id="map")

            #print(tf_xyz_box.shape)
            #self.pc_pub.publish(tf_pc_msg)
            #tf_path_points_msg = self.xyz_array_to_pointcloud2(path_points_xyz, frame_id="map")
            #self.pp_pub.publish(tf_path_points_msg)
            #print(time.time() - t1)
            print(time.time() - t1)
            path = self.points_to_path(local_path)
            self.local_path_pub.publish(path)
            tf_path_points_msg = self.xyz_array_to_pointcloud2(path_points_xyz, frame_id="map")
            self.pp_pub.publish(tf_path_points_msg)
            tf_pc_msg = self.xyz_array_to_pointcloud2(tf_xyz_box, frame_id="map")
            self.pc_pub.publish(tf_pc_msg)
            self._result.z = 5
            self._result.local_path = path_msg
            self._result.got_terrain = True
            self._as.set_succeeded(self._result)

        else:
            self._result.z = goal.z
            self._result.local_path = path_msg
            self._result.got_terrain = False
            self._as.set_succeeded(self._result)

    def repulse(self, points, terrain):
        new_ponits = []
        for point in points:
            pts = np.vstack((point, terrain))
            tree = KDTree(pts, leaf_size=2)
            dist, idx = tree.query(pts[:1], k=2)
            cp = pts[idx[0][-1]]  # closest point
            dir = point - cp
            dist = np.linalg.norm(dir)
            dir = dir/dist
            d = 2*np.exp(-dist**2/2)
            new_point = point + d * dir
            new_ponits.append(new_point)
        return np.asarray(new_ponits)


    def findPathPoint(self, terrain, point, height):
        """
        1) find the closest point P in terrain given point; 2) estimate normal of P;
        3) get path point position along the normal;
        :param terrain:
        :param point:
        :param height:
        :return:
        """
        pts = np.vstack((point, terrain))
        tree = KDTree(pts, leaf_size=2)
        dist, idx = tree.query(pts[:1], k=2)
        cp = pts[idx[0][-1]]  # closest point
        self.terrain_points.append(cp)
        tree = KDTree(terrain, leaf_size=2)
        dist, idx = tree.query(cp.reshape(1, -1), k=20)
        normal = self.normalEstimation(dist, terrain[idx])
        point = cp + normal * height
        return point, normal

    def addPathPoint(self, terrain, path_points, point, normal):
        if len(path_points) == 0:
            """
            tangent = np.asarray((normal[1], -normal[0]))
            if tangent[0] > 0:
                path_points.append({'p': point, 't': tangent, 'n': normal})
            elif tangent[0] < 0:
                path_points.append({'p': point, 't': -tangent, 'n': normal})
            else:
                if point[0] <= 0:
                    path_points.append({'p': point, 't': np.asarray((0, -1)), 'n': normal})
                else:
                    path_points.append({'p': point, 't': np.asarray((0, 1)), 'n': normal})
            """
            dir = np.asarray((0, 1, 0))
            normal_xyz = np.asarray((normal[0], 0, normal[1]))
            tangent_xyz = np.cross(dir, normal_xyz)
            tangent = np.asarray((tangent_xyz[0], tangent_xyz[2]))
            path_points.append({'p': point, 't': tangent, 'n': normal})
            return True


        if len(path_points) == 20:
            path_points.append({'p': point, 't': path_points[-1]['t'], 'n': path_points[-1]['n']})
            return False

        prev_point = path_points[-1]['p']
        prev_normal = path_points[-1]['n']

        if (np.linalg.norm(prev_point - point) == 0) & (np.linalg.norm(prev_normal - normal) == 0):
            return False

        prev_tangent = path_points[-1]['t']
        prev_tangent_xyz = np.asarray((prev_tangent[0], 0, prev_tangent[1]))
        prev_normal_xyz = np.asarray((prev_normal[0], 0, prev_normal[1]))
        dir = np.cross(prev_normal_xyz, prev_tangent_xyz)
        normal_xyz = np.asarray((normal[0], 0, normal[1]))
        tangent_xyz = np.cross(dir, normal_xyz)
        tangent = np.asarray((tangent_xyz[0], tangent_xyz[2]))
        path_points.append({'p': point, 't': tangent, 'n': normal})
        return True

    def predictPathPoint(self, terrain, path_points, step):
        point = path_points[-1]['p']
        tangent = path_points[-1]['t']
        pred_point = point + tangent * step
        return pred_point

    def generatePathPoints(self, terrain, height):
        cur_pos = np.asarray((0, 0))
        path_points = []
        self.terrain_points = []
        point, normal = self.findPathPoint(terrain, cur_pos, height)
        while self.addPathPoint(terrain, path_points, point, normal):
            pred_point = self.predictPathPoint(terrain, path_points, step=0.3)  # step must be smaller than height
            # otherwise, pred_point might go into the terrain
            point, normal = self.findPathPoint(terrain, pred_point, height)
        print(len(path_points))
        return path_points


    def FFT_LPF(self, image, wx, wy):
        dft = cv2.dft(np.float32(image), flags=cv2.DFT_COMPLEX_OUTPUT)
        dft_shift = np.fft.fftshift(dft)
        magnitude_spectrum = 20 * np.log(cv2.magnitude(dft_shift[:, :, 0], dft_shift[:, :, 1]))
        rows, cols = image.shape
        crow, ccol = rows / 2, cols / 2
        # create a mask first, center square is 1, remaining all zeros
        mask = np.zeros((rows, cols, 2), np.uint8)
        mask[crow - wx:crow + wx, ccol - wy:ccol + wy] = 1
        fshift = dft_shift * mask
        f_ishift = np.fft.ifftshift(fshift)
        img_back = cv2.idft(f_ishift)
        img_back = cv2.magnitude(img_back[:, :, 0], img_back[:, :, 1])
        return img_back

    def gridify(self, points, spacing):
        return np.round(points/spacing).astype(int)

    def findPathPoints(self, point, centers, radius):
        """
        findPathPoints is a recursive function that finds poles of the closest circles wrt point
        :param point:
        :param centers:
        :param radius:
        :return:
        """
        if centers.shape[0] == 1:
            return centers[0]
        else:
            # find closest circle
            p = np.vstack((point, centers))
            tree = KDTree(p, leaf_size=2)
            dist, idx = tree.query(p[:1], k=2)
            center = centers[idx[0][-1], :]
            # find the pole
            pole = self.findPole(point, center, radius)
            next_point = center
            next_centers = np.delete(centers, idx[0][-1]-1, axis=0)
            return pole, self.findPathPoints(next_point, next_centers, radius)

    def findPole(self, point, center, radius):
        """
        find a circle pole that is closer to the origin (robot)
        :param point:
        :param center:
        :param radius:
        :return:
        """
        print('x')
        print(point)
        print(center)
        # move the diagram such that the circle center is at origin
        point = point - center
        # compute the angle between PC and x-axis
        alpha = math.atan2(point[1], point[0])
        # rotate the diagram such that P is on x-axis
        R = np.asarray(((math.cos(alpha), -math.sin(alpha)), (math.sin(alpha), math.cos(alpha))))
        point = np.matmul(np.linalg.inv(R), point)
        # get the conic format of the circle
        C = np.asarray(((1, 0, 0), (0, 1, 0), (0, 0, -radius**2)))
        # get point in homogeneous coords
        point = np.asarray((point[0], point[1], 1))
        # get a line that passes two tangent points
        line = np.matmul(point, C)
        # get two poles
        x = -line[2]/line[0]
        print(radius**2 - x**2)
        y1 = math.sqrt(radius**2 - x**2)
        y2 = -y1
        p1 = np.asarray((x, y1))
        p2 = np.asarray((x, y2))
        # move points to original coords
        p1 = np.matmul(R, p1) + center
        p2 = np.matmul(R, p2) + center
        if np.linalg.norm(p1) < np.linalg.norm(p2):
            return p1
        else:
            return p2

    def getNormals(self, query_points, points):
        """
        get normals of query_points in points
        :param points:
        :return:
        """
        tree = KDTree(points, leaf_size=4)
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
        P = -points[0, :]

        if np.dot(N, P) < 0:
            return -N
        else:
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

    def points_to_path(self, points):
        poses = []
        for point in points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            poses.append(pose)
        path = Path()
        path.poses = poses
        path.header.frame_id = 'map'

        return path

    def cart2pol(self, x, y):
        rho = np.sqrt(x ** 2 + y ** 2)
        phi = np.arctan2(y, x)
        return (phi, rho)

    def pol2cart(self, phi, rho):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return (x, y)

if __name__ == "__main__":
    rospy.init_node("perception_model", anonymous=False)
    pm = Perception_Model()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")


