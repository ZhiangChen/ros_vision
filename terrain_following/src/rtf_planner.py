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


class RTF_Planner(PosControl):
    def __init__(self, height, radius=0.2):
        super(RTF_Planner, self).__init__(radius)
        self.height = height

        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        self.reach_position(0, 0, self.height, 30)
        self.reach_position(0, 10, self.height, 30)
        self.reach_position(10, 10, self.height, 30)
        self.reach_position(10, 0, self.height, 30)
        self.reach_position(0, 0, self.height, 30)



if __name__ == '__main__':
    rospy.init_node('rtf_controller', anonymous=True)
    controller = RTF_Planner(4)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")