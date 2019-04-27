#!/usr/bin/env python2
"""
Zhiang Chen
April 2019
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, \
                            WaypointPush
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix


import mavros_controller as MController



class Mavros_Velocity_Controller(MController):
    def __init__(self):
        super(MController, self).__init__()


        




if __name__ == '__main__':
    rospy.init_node('vlc_test_node', anonymous=True)
    controller = Mavros_Velocity_Controller()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")