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
from tf.transformations import euler_from_quaternion
from mavros_controller import Mavros_Controller as MController

import actionlib
import terrain_following.msg

TIME_OUT = 60*20

class Mavros_Velocity_Controller(MController):
    def __init__(self, radius=0.15, step_size=0.3):
        super(Mavros_Velocity_Controller, self).__init__()

        # controller parameters
        self.radius = radius
        self.K_xy_I = 0.3  # xy integral gain
        self.K_xy_P = 0.5  # xy proportional gain
        self.K_z_P = 1.0  # z proportional gain
        self.K_P = 0.3  # proportional gain for approaching stage or takeoff
        self.K_I = 0.1  # integral gain for approaching stage or takeoff
        self.K_angle_P = 0.1
        self.step_size = step_size
        self._xy_err_old = np.zeros(2)
        self._xyz_err_old = np.zeros(3)
        self._desired_angle_z_old = 0

        self._takeoff = False
        self._adjust_head = False
        self._turning_direction = False
        self._old_position = np.zeros(3)

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

        self.__velctl(waypoints)

        #feedback = terrain_following.msg.waypointsFeedback()
        result = terrain_following.msg.waypointsResult()
        result.success = True
        self._as.set_succeeded(result)

    def send_vel(self):
        Hz = 30
        rate = rospy.Rate(Hz)  # Hz
        self.vel.header = Header()
        self.vel.header.frame_id = "base_footprint"
        vel_max = np.ones(3)*0.5
        vel_min = -np.ones(3)*0.5

        while not rospy.is_shutdown():
            desire_xyz = np.array((self.pos.pose.position.x, self.pos.pose.position.y, self.pos.pose.position.z))
            current_xyz = np.array((self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z))
            current_angle_z = np.array(euler_from_quaternion((self.local_position.pose.orientation.x, self.local_position.pose.orientation.y, self.local_position.pose.orientation.z, self.local_position.pose.orientation.w)))[2]
            err = desire_xyz - current_xyz
            desire_angle_z = math.atan2(err[1], err[0])
            err_angle_z = desire_angle_z - current_angle_z
            if not self._takeoff:
                control_err = err
                vel = control_err * self.K_P + self._xyz_err_old * self.K_I
                vel = np.max((vel, vel_min), axis=0)
                vel = np.min((vel, vel_max), axis=0)
                self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z = vel[0], vel[1], vel[2]
                self.vel.header.stamp = rospy.Time.now()
                self.vel_setpoint_pub.publish(self.vel)
                self._xyz_err_old = control_err
                rate.sleep()

            elif not self._adjust_head :
                control_err = self._old_position - current_xyz
                vel = control_err * self.K_P + self._xyz_err_old * self.K_I
                self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z = vel[0], vel[1], vel[2]

                # to get rid of singularity angle
                if desire_angle_z - self._desired_angle_z_old > 5:
                    desire_angle_z -= 3.14*2
                elif desire_angle_z - self._desired_angle_z_old < -5:
                    desire_angle_z += 3.14*2

                err_angle_z = desire_angle_z - current_angle_z

                if not self._turning_direction:
                    if err_angle_z > 0:
                        self._turning_direction = 1
                    else:
                        self._turning_direction = -1

                if abs(err_angle_z) < 0.5:
                    angle_vel_z = err_angle_z * self.K_angle_P
                else:
                    angle_vel_z = abs(err_angle_z * self.K_angle_P) * self._turning_direction

                angle_vel_z = np.max((angle_vel_z, -0.2), axis=0)
                angle_vel_z = np.min((angle_vel_z, 0.2), axis=0)
                self.vel.twist.angular.z = angle_vel_z
                self.vel.header.stamp = rospy.Time.now()
                self.vel_setpoint_pub.publish(self.vel)
                self._xyz_err_old = control_err
                self._desired_angle_z_old = desire_angle_z
                rate.sleep()
                if abs(err_angle_z) < 0.02:
                    self.vel.twist.angular.z = 0
                    self._adjust_head = True
                    self._turning_direction = False
                    self._xyz_err_old = np.zeros(3)

            elif np.linalg.norm(err) > self.radius*4:
                xy_err = err[:2]
                xy_step_size = xy_err/np.linalg.norm(xy_err) * self.step_size
                next_xy = current_xyz[:2] + xy_step_size
                next_xyz = np.append(next_xy, current_xyz[2])
                self._goal.x, self._goal.y, self._goal.z = next_xy[0], next_xy[1], current_xyz[2]
                self._goal.x_des, self._goal.y_des = desire_xyz[0], desire_xyz[1]
                self._perception_client.send_goal(self._goal)
                self._perception_client.wait_for_result(rospy.Duration.from_sec(1.0 / Hz))
                # print(self._perception_client.get_state())
                if self._perception_client.get_state() == 3:  # 3: SUCCEED
                    result = self._perception_client.get_result()
                    if result is not None:
                        next_xyz[2] = result.z
                        if not result.got_terrain:
                            print(result)
                z_vel = (next_xyz[2] - current_xyz[2]) * self.K_z_P
                xy_vel = xy_step_size * self.K_xy_P + (xy_step_size + self._xy_err_old) * self.K_xy_I
                vel = np.append(xy_vel, z_vel)
                vel = np.max((vel, vel_min), axis=0)
                vel = np.min((vel, vel_max), axis=0)
                self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z = vel[0], vel[1], vel[2]
                self.vel.header.stamp = rospy.Time.now()
                self.vel_setpoint_pub.publish(self.vel)
                self._xy_err_old = xy_step_size

            elif self.radius*4 >= np.linalg.norm(err) >= self.radius:
                step_size = err/np.linalg.norm(err) * self.step_size
                next_xyz = current_xyz + step_size
                self._goal.x, self._goal.y, self._goal.z = next_xyz
                self._goal.x_des, self._goal.y_des = desire_xyz[0], desire_xyz[1]
                self._perception_client.send_goal(self._goal)
                self._perception_client.wait_for_result(rospy.Duration.from_sec(1.0/Hz))

                if self._perception_client.get_state() == 3:  # 3: SUCCEED
                    result = self._perception_client.get_result()
                    if type(result) is not None:
                        next_xyz[2] = result.z

                control_err = next_xyz - current_xyz
                vel = control_err * self.K_P + self._xyz_err_old * self.K_I
                vel = np.max((vel, vel_min), axis=0)
                vel = np.min((vel, vel_max), axis=0)
                self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z = vel[0], vel[1], vel[2]
                self.vel.header.stamp = rospy.Time.now()
                self.vel_setpoint_pub.publish(self.vel)
                self._xyz_err_old = control_err
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
                self._old_position = np.array((self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z))
                self._xy_err_old = np.zeros(2)
                self._xyz_err_old = np.zeros(3)
                self._adjust_head = False
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

    def __velctl(self, positions):
        """Test offboard velocity control"""

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
            if i > 0:
                self._takeoff = True
            self.reach_waypoint(positions[i][0], positions[i][1],
                                positions[i][2], TIME_OUT)

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
