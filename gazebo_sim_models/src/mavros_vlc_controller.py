#!/usr/bin/env python2
"""
Zhiang Chen
April 2019
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
from mavros_controller import Mavros_Controller as MController



class Mavros_Velocity_Controller(MController):
    def __init__(self):
        super(Mavros_Velocity_Controller, self).__init__()

        self.vel = TwistStamped()
        self.radius = 0.1 # velocity error tolerance

        self.vel_setpoint_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)

        self.vel_thread = Thread(target=self.send_vel, args=())
        self.vel_thread.daemon = True
        self.vel_thread.start()

        self.test_velctl()

    def send_vel(self):
        rate = rospy.Rate(10)  # Hz
        self.vel.header = Header()
        self.vel.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.vel.header.stamp = rospy.Time.now()
            self.vel_setpoint_pub.publish(self.vel)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


    def reach_velocity(self, velocity, timeout):
        self.vel.twist = velocity.twist
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_velocity():
                rospy.loginfo("velocity reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)


    def is_at_velocity(self):
        desired = np.array((self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z,
                            self.vel.twist.angular.x, self.vel.twist.angular.y, self.vel.twist.angular.z))

        current = np.array((self.local_vel.twist.linear.x, self.local_vel.twist.linear.y, self.local_vel.twist.linear.z,
                            self.local_vel.twist.angular.x, self.local_vel.twist.angular.y, self.local_vel.twist.angular.z))

        error = np.linalg.norm(desired - current)

        return error < self.radius



    def test_velctl(self):
        """Test offboard velocity control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        velocity = TwistStamped()
        velocity.twist.linear.z = 2.0

        self.reach_velocity(velocity, 30)


        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)
        rospy.loginfo("MISSION COMPLETE")







if __name__ == '__main__':
    rospy.init_node('vlc_test_node', anonymous=True)
    controller = Mavros_Velocity_Controller()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")