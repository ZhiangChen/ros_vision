#!/usr/bin/env python2
"""
Zhiang Chen
April 2019
"""

import rospy
import numpy as np
import math
from mavros_pos_controller import Mavros_Position_Controller as PosControl
from pymavlink import mavutil
from perception_model import Depth_Camera


class RTF_Planner(PosControl):
    def __init__(self, height, radius=0.2, ahead_dist = 1):
        super(RTF_Planner, self).__init__(radius)
        self.height = height
        self.ahead_dist = ahead_dist
        self.realsense = Depth_Camera()

    def reach_position(self, x, y, z, timeout):
        """
        overwrite reach_position
        :param x:
        :param y:
        :param z:
        :param timeout:
        :return:
        """
        """timeout(int): seconds"""
        # set a position setpoint

        if self.local_position.pose.position.z < 1:  # takeoff first
            self.pos.pose.position.x = self.local_position.pose.position.x
            self.pos.pose.position.y = self.local_position.pose.position.y
            self.pos.pose.position.z = self.height + self.local_position.pose.position.z
            rospy.loginfo("taking off")
            reached = self.wait_for_position(timeout)
            if reached:
                rospy.loginfo("reached the height")

        desired_xy = np.array((x, y))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y))

        dist = np.linalg.norm(desired_xy - pos)
        step = 0
        N = int(dist/self.radius)+100

        while dist > self.ahead_dist:
            step = step + 1
            if step > N:
                rospy.logerr("out of steps")
                break
            current_xy = np.array((self.local_position.pose.position.x, self.local_position.pose.position.y))
            direction = desired_xy - current_xy
            gain = direction/np.linalg.norm(direction) * self.ahead_dist * 2
            next_xy = current_xy + gain
            terrain_height = self.realsense.estimate_terrain_vertical_distance(next_xy, k=5)

            self.pos.pose.position.x = next_xy[0]
            self.pos.pose.position.y = next_xy[1]
            self.pos.pose.position.z = self.height + terrain_height


            self.wait_for_position(timeout)
            pos = np.array((self.local_position.pose.position.x,
                            self.local_position.pose.position.y))

            dist = np.linalg.norm(desired_xy - pos)

        terrain_height = self.realsense.estimate_terrain_vertical_distance((self.local_position.pose.position.x,
                                                                            self.local_position.pose.position.y), k=5)

        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}| current position x: {2:.2f}, y: {3:.2f}, z: {4:.2f}".
                format(x, y, self.local_position.pose.position.x,
                       self.local_position.pose.position.y,
                       terrain_height))




    def wait_for_position(self, timeout):
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        return reached





if __name__ == '__main__':
    rospy.init_node('rtf_controller', anonymous=True)
    controller = RTF_Planner(3)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")