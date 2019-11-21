#!/usr/bin/env python
"""
Zhiang Chen
Nov 2019
"""

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class Image_Processor(object):
    def __init__(self):
        rospy.Subscriber("/r200/depth/image_raw", Image, self.record_callback)
        self.delta_time = 0
        self.bridge = CvBridge()
        print("Image processor node initialized!")

    def start_recording(self, save_path, delta_time=1):
        self.save_path = save_path
        self.frame = 0
        self.delta_time = delta_time

    def record_callback(self, data):
        print("callback works")
        if self.delta_time >0:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print(image.shape)
            cv2.imwrite(self.save_path + str(self.frame) + ".png", image)
            rospy.sleep(self.delta_time)
            self.frame += 1

if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    IMP = Image_Processor()
    IMP.start_recording("/home/zhiang/Pictures/terrain_boulder/")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")