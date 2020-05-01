#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
offboard_ctrl.py:

"""

###############################################
# Standard Imports                            #
###############################################
import time
import threading
from math import *
import numpy as np

###############################################
# ROS Imports                                 #
###############################################
import rospy
import rospkg

###############################################
# ROS Topic messages                          #
###############################################
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan

###############################################
# ROS Service messages                        #
###############################################
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

###############################################
# Own classes				      #
###############################################
from Lidar_plugin import *


class rotocraft(object):
    def __init__(self):
        self.pos = np.array([[0.0],[0.0],[0.0]])
        self.attitude = np.array([[0.0],[0.0],[0.0]])
        self.vel = np.array([[0.0],[0.0],[0.0]])

        self.target_pos = np.array([[5],[5],[2]])
        self.target_vel = np.array([[0],[0],[0]])
        self.error_pos = np.array([[0],[0],[0]])
        self.error_vel = np.array([[0],[0],[0]])

        self.ns = ""

        #self.lidar_range = np.zeros(100)
        #self.lidar_angle_dat = np.zeros(3)

        self.U = np.array([[0],[0],[0]])    # Accelerations

        rospy.Subscriber(self.ns+"/mavros/local_position/velocity_local", TwistStamped, self.cb_vel)
        rospy.Subscriber(self.ns+"/mavros/local_position/pose", PoseStamped, self.cb_pos)
        self.lidarSensor = Lidar_plugin(self.ns)

    """
    Callbacks
    * cb_state
    * cb_target
    """
    def cb_vel(self,data):
        print("cb_vel")
        self.vel[0][0] = data.twist.linear.x
        self.vel[1][0] = data.twist.linear.y
        self.vel[2][0] = data.twist.linear.z

    def cb_pos(self,data):
        print("cb_pos")
        self.pos[0][0] = data.pose.position.x
        self.pos[2][0] = data.pose.position.z
        self.pos[1][0] = data.pose.position.y

    """def cb_lidar(self,data):
        print("Hey Lidar data!:")
        # set angle data {min, step, max} [rad]
        self.lidar_angle_dat[0] = data.angle_min
        self.lidar_angle_dat[1] = data.angle_increment
        self.lidar_angle_dat[2] = data.angle_max

        # set range [m]
        self.lidar_range = data.ranges
        print(str(self.lidar_range))
    """

class traj_grp1(object):
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        self.err = 0.0
        self.grad = np.array([[0.0],[0.0]])
        self.rc = rotocraft()
        self.vel_goal = np.array([[0.0],[0.0]])
        self.Ugvf = np.array([[0.0],[0.0],[15.0]])

        # Circle param
        self.centerCirc = np.array([[0.0],[0.0]])
        self.radius = 2.0

        # Regulation constants
        self.c_vel= 1
        self.c_pos = 1
        self.c_follow = 1

        # 90 deg Rot_Matrix
        self.rotM = np.array([[0.0, -1.0],[1.0, 0.0]])

        self.state = "INIT"

        self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

        self.local_vel_pub = rospy.Publisher(self.rc.ns+"/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

        # Init msgs
        self.target = TwistStamped()
        self.target.twist.linear.x = 0
        self.target.twist.linear.y = 0
        self.target.twist.linear.z = 0

        ## Create services
        self.setpoint_controller_server()

        self.t_circle = threading.Thread(target=self.circle)
        self.t_circle.start()
        print(">> Starting circle (Thread)")

        rospy.spin()

    """
    State
    * calc_err:
    * update_grad_circle:
    * update_U:
    """
    def calc_err(self):
        self.err = (self.rc.pos[0][0]-self.centerCirc[0][0])**2 + (self.rc.pos[1][0]-self.centerCirc[1][0])**2 -self.radius**2

    def update_grad_circle(self):
        self.grad = 2.0*(self.rc.pos[0:2]-self.centerCirc[0:2])

    def update_U(self):
        #print("c_vel: {}".format(self.rotM.dot(self.grad)))
        self.Ugvf = -self.c_pos*self.err*self.grad - self.c_vel*self.rc.vel[0:2]+self.c_follow*self.rotM.dot(self.grad)


    """
    Services
    *
    """
    def setpoint_controller_server(self):
        s_circle = rospy.Service('setpoint_controller/circle', Empty, self.start_circle)
        s_stop = rospy.Service('setpoint_controller/stop', Empty, self.stop)

        print("The SetPoint Controller is ready")

    """
    State
    * set_state:
    * get_state_
    """
    def set_state(self, data):
        self.state = data
        print("New State: {}".format(data))

    def get_state(self):
        return self.state

    """
    Circle
    """
    def start_circle(self, r):
        self.t_circle = threading.Thread(target=self.circle)
        self.t_circle.start()
        print(">> Starting circle (Thread)")

        return {}

    def circle(self):
        self.set_state("CIRCLE")
        #print("lidar dist: \t"+str(self.rc.lidarSensor.get_minDist()))

        while self.state == "CIRCLE":
            print("lidar dist: \t"+str(self.rc.lidarSensor.get_minRange_angles()))
            self.target.header.frame_id = "base_footprint"
            self.target.header.stamp = rospy.Time.now()

            self.calc_err()
            self.update_grad_circle()
            self.update_U()

            # print(self.Ugvf)
            self.target.twist.linear.x = self.Ugvf[0][0] * 0.5
            self.target.twist.linear.y = self.Ugvf[1][0] * 0.5

            self.local_vel_pub.publish(self.target)
            self.rate.sleep()

        print(">> Cicle has stopped")
        self.target.twist.linear.x = 0
        self.target.twist.linear.y = 0
        self.target.twist.linear.z = 0

        self.local_vel_pub.publish(self.target)

    """
    Stop
    """
    def stop(self,r):
        self.set_state("STOP")
        return {}

if __name__ == '__main__':
    Tj = traj_grp1()
