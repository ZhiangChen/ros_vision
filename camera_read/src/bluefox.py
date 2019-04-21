#!/usr/bin/env python
"""
Zhiang Chen, April 2019
"""

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


class Camera(object):
    def __init__(self):
        rospy.Subscriber("/mv_26806037/image_raw", Image, self.callback)
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        self.pub1 = rospy.Publisher("/camera_reader/green_image", Image, queue_size=1)
        self.pub2 = rospy.Publisher("/camera_reader/mean_image", Image, queue_size=1)
        self.pub3 = rospy.Publisher("/imu", Imu, queue_size=1)
        self.bridge = CvBridge()
        print("ROS Node Initialized: bluefox_reader")

    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        mean_image = self.bridge.cv2_to_imgmsg(np.average(image, axis=2).astype("uint8"), encoding="mono8")
        green_image = self.bridge.cv2_to_imgmsg(image[:,:,1], encoding="mono8")
        green_image.header.stamp = data.header
        mean_image.header = data.header
        self.pub1.publish(green_image)
        self.pub2.publish(mean_image)

    def imu_callback(self, data):
        imu_data = data
        imu_data.header.stamp = rospy.Time.now()
        self.pub3.publish(imu_data)
        rospy.sleep(0.005)
        #update time
        imu_data.header.stamp = rospy.Time.now()
        self.pub3.publish(imu_data)
        rospy.sleep(0.005)
        # update time
        imu_data.header.stamp = rospy.Time.now()
        self.pub3.publish(imu_data)
        rospy.sleep(0.005)
        # update time
        imu_data.header.stamp = rospy.Time.now()
        self.pub3.publish(imu_data)






if __name__ == "__main__":
    rospy.init_node("bluefox_reader", anonymous=False)
    camera_reader = Camera()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")