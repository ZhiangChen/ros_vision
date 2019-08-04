#!/usr/bin/env python2
"""
Zhiang Chen
May 2019
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

import actionlib
import terrain_following.msg


class Mavros_Velocity_Controller(MController):
    def __init__(self, radius=0.2, K=0.3, step_size=0.8):
        super(Mavros_Velocity_Controller, self).__init__()

        self.radius = radius
        self.K = K
        self.step_size = step_size
        self._takeoff = False

        self._as = actionlib.SimpleActionServer("waypoints_mission", terrain_following.msg.waypointsAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("waypoints_server started.")

        self._perception_client = actionlib.SimpleActionClient("terrain_lookup", terrain_following.msg.terrainAction)
        self._perception_client.wait_for_server()
        self._goal = terrain_following.msg.terrainGoal()

        self.vel = TwistStamped()
        self.pos = self.local_position
        self.vel_setpoint_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)

        self.vel_thread = Thread(target=self.send_vel, args=())
        self.vel_thread.daemon = True
        self.vel_thread.start()

        rospy.loginfo("Mavros_Velocity_Controller initialized")


    def execute_cb(self, goal):
        try:
            waypoints = np.fromstring(goal.waypoints).reshape((-1, 3))
        except:
            rospy.logerr("Invalid waypoints")

        self.__posctl(waypoints)

        #feedback = terrain_following.msg.waypointsFeedback()
        result = terrain_following.msg.waypointsResult()
        result.success = True
        self._as.set_succeeded(result)

    def send_vel(self):
        Hz = 30
        rate = rospy.Rate(Hz)  # Hz
        self.vel.header = Header()
        self.vel.header.frame_id = "base_footprint"
        vel_max = np.ones(3)*1
        vel_min = -np.ones(3)*1

        while not rospy.is_shutdown():
            desire_xyz = np.array((self.pos.pose.position.x, self.pos.pose.position.y, self.pos.pose.position.z))
            current_xyz = np.array((self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z))
            err = desire_xyz - current_xyz
            if not self._takeoff:
                control_err = err
                vel = control_err * self.K
                vel = np.max((vel, vel_min), axis=0)
                vel = np.min((vel, vel_max), axis=0)
                self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z = vel[0], vel[1], vel[2]
                self.vel.header.stamp = rospy.Time.now()
                self.vel_setpoint_pub.publish(self.vel)
                rate.sleep()

            elif np.linalg.norm(err) > self.radius*4:
                xy_err = err[:2]
                xy_step_size = xy_err/np.linalg.norm(xy_err) * self.step_size
                next_xy = current_xyz[:2] + xy_step_size
                next_xyz = np.append(next_xy, current_xyz[2])
                self._goal.x, self._goal.y, self._goal.z = next_xy[0], next_xy[1], current_xyz[2]

                self._perception_client.send_goal(self._goal)
                self._perception_client.wait_for_result(rospy.Duration.from_sec(1.0 / Hz))
                # print(self._perception_client.get_state())
                if self._perception_client.get_state() == 3:  # 3: SUCCEED
                    result = self._perception_client.get_result()
                    if result is not None:
                        next_xyz[2] = result.z
                        print(result)

                control_err = next_xyz - current_xyz
                vel = control_err * self.K
                vel = np.max((vel, vel_min), axis=0)
                vel = np.min((vel, vel_max), axis=0)

                self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z = vel[0], vel[1], vel[2]
                self.vel.header.stamp = rospy.Time.now()
                self.vel_setpoint_pub.publish(self.vel)

            elif self.radius*4 >= np.linalg.norm(err) >= self.radius:
                step_size = err/np.linalg.norm(err) * self.step_size
                next_xyz = current_xyz + step_size
                self._goal.x, self._goal.y, self._goal.z = next_xyz
                self._perception_client.send_goal(self._goal)
                self._perception_client.wait_for_result(rospy.Duration.from_sec(1.0/Hz))

                if self._perception_client.get_state() == 3:  # 3: SUCCEED
                    result = self._perception_client.get_result()
                    if type(result) is not None:
                        next_xyz[2] = result.z


                control_err = next_xyz - current_xyz
                vel = control_err * self.K
                vel = np.max((vel, vel_min), axis=0)
                vel = np.min((vel, vel_max), axis=0)
                self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z = vel[0], vel[1], vel[2]
                self.vel.header.stamp = rospy.Time.now()
                self.vel_setpoint_pub.publish(self.vel)
            else:
                rate.sleep()


    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))
        if self._takeoff:
            desired = np.array((x, y))
            pos = np.array((self.local_position.pose.position.x,
                            self.local_position.pose.position.y))
        else:
            desired = np.array((x, y, z))
            pos = np.array((self.local_position.pose.position.x,
                            self.local_position.pose.position.y,
                            self.local_position.pose.position.z))

        return np.linalg.norm(desired - pos) < offset

    def reach_waypoint(self, x, y, z, timeout):
        """timeout(int): seconds"""

        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    def __posctl(self, positions):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")

        self.relative_height = positions[0][2]
        self._goal.relative_height = positions[0][2]
        for i in xrange(len(positions)):
            if i>0:
                self._takeoff = True
            self.reach_waypoint(positions[i][0], positions[i][1],
                                positions[i][2], 600)

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self._takeoff = False
        self.set_arm(False, 5)
        rospy.loginfo("MISSION COMPLETE")




if __name__ == '__main__':
    rospy.init_node('vel_controller_node', anonymous=True)
    controller = Mavros_Velocity_Controller()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")
