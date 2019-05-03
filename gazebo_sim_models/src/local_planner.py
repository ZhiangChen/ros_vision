#!/usr/bin/env python2
"""
Zhiang Chen
May 2019
"""

class Local_Planner(object):
    def __init__(self):
        pass



if __name__ == '__main__':
    rospy.init_node('local_planner', anonymous=True)
    planner = Mavros_Position_Controller()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")
