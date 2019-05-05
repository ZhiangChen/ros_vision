#!/usr/bin/env python2
"""
Zhiang Chen
April 2019
"""

import rospy
import numpy as np
import math
from mavros_pos_controller2 import Mavros_Position_Controller as PosControl
from pymavlink import mavutil
from perception_model import Depth_Camera
from std_msgs.msg import Float32



class RTF_Planner(PosControl):
    def __init__(self, height, radius=0.4):
        super(RTF_Planner, self).__init__(radius)
        self.height = height
        #self.realsense = Depth_Camera()  # bad. Do not have too many efficient subscribers

        self.pub_terrain_height = rospy.Publisher("/terrain_height", Float32, queue_size=1)


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
        N = int(dist/self.radius)+10000

        while dist > self.radius:

            step = step + 1
            if step > N:
                rospy.logerr("out of steps")
                break

            current_xy = np.array((self.local_position.pose.position.x, self.local_position.pose.position.y))
            direction = desired_xy - current_xy


            gain = direction/np.linalg.norm(direction) * self.radius * 1.9
            next_xy = current_xy + gain
            #terrain_height = self.realsense.estimate_terrain_vertical_distance(next_xy, k=5)
            terrain_height = 0


            next_z = self.height + terrain_height
            #when math.fabs(self.local_position.pose.position.z - next_z) >

            self.pos.pose.position.x = next_xy[0]
            self.pos.pose.position.y = next_xy[1]
            self.pos.pose.position.z = next_z

            #self.pub_terrain_height.publish(next_z)

            #self.wait_for_position(timeout)
            rospy.sleep(0.1)
            pos = np.array((self.local_position.pose.position.x,
                            self.local_position.pose.position.y))

            dist = np.linalg.norm(desired_xy - pos)

        #terrain_height = self.realsense.estimate_terrain_vertical_distance((self.local_position.pose.position.x, self.local_position.pose.position.y), k=5)
        terrain_height = 0

        rospy.loginfo(
            "desired position | x: {0}, y: {1}| current position x: {2:.2f}, y: {3:.2f}, terrain_height: {4:.2f}".
                format(x, y, self.local_position.pose.position.x,
                       self.local_position.pose.position.y,
                       terrain_height))




    def wait_for_position(self, timeout):
        loop_freq = 20  # Hz
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