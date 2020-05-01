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
#import skfuzzy as fuzz
#from skfuzzy import control as ctrl
#import matplotlib.pyplot as plt

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
from mavros_msgs.msg import PositionTarget

###############################################
# ROS Service messages                        #
###############################################
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

##############################################
# std import
##
from Lidar_plugin import Lidar_plugin

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
        self.lidar = Lidar_plugin(self.ns)

        self.U = np.array([[0],[0],[0]])    # Accelerations

        rospy.Subscriber(self.ns+"/mavros/local_position/velocity_local", TwistStamped, self.cb_vel)
        rospy.Subscriber(self.ns+"/mavros/local_position/pose", PoseStamped, self.cb_pos)

        self.yaw = 0

    """
    Callbacks
    * cb_state
    * cb_target
    """
    def cb_vel(self,data):
        self.vel[0][0] = data.twist.linear.x
        self.vel[1][0] = data.twist.linear.y
        self.vel[2][0] = data.twist.linear.z

    def cb_pos(self,data):
        self.pos[0][0] = data.pose.position.x
        self.pos[1][0] = data.pose.position.y
        self.pos[2][0] = data.pose.position.z

class traj_grp1(object):
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        self.err = 0.0
        self.grad = np.array([[0.0],[0.0]])
        self.hess = np.array([[2.0,0.0],[0.0,2.0]])
        self.rc = rotocraft()
        self.vel_goal = np.array([[0.0],[0.0]])
        self.Ugvf = np.array([[0.0],[0.0],[15.0]])

        # Circle param
        self.centerCirc = np.array([[0.0],[0.0]])
        self.radius = 20.0

        # Regulation constants
        self.c_pos = 0.005
        self.c_vel= 3.0
        self.c_acc = 1.25

        self.s = 3.00
        self.count = 0
        # 90 deg Rot_Matrix
        self.rotM = np.array([[0.0, -1.0],[1.0, 0.0]])

        self.state = "INIT"

        self.rate = rospy.Rate(20.0) # MUST be more then 2Hz


        #self.local_vel_pub = rospy.Publisher(self.rc.ns+"/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.local_pos_pub = rospy.Publisher(self.rc.ns+"/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        '''
        # Init msgs
        self.target = TwistStamped()
        self.target.twist.linear.x = 0
        self.target.twist.linear.y = 0
        self.target.twist.linear.z = 0
        self.target.twist.angular.z = 0
        '''

        self.pos_target = PositionTarget()
        self.pos_target.velocity.x = 0
        self.pos_target.velocity.y = 0
        self.pos_target.velocity.z = 0
        self.pos_target.position.z = 2
        self.pos_target.yaw = 0


        ## Create services
        self.setpoint_controller_server()

        self.t_circle = threading.Thread(target=self.circle)
        #self.t_plot = threading.Thread(target=self.plotVectors)
        self.t_circle.start()
        #self.t_plot.start()
        """
        self.dist = ctrl.Antecedent(np.arange(0, 30, 1), 'dist')
        self.change_in_r = ctrl.Consequent(np.arange(-5, 5, 1), 'change_in_r')

        # Auto-membership function population is possible with .automf(3, 5, or 7)
        self.dist.automf(3)

        #fuzzy
        self.change_in_r['low'] = fuzz.trimf(self.change_in_r.universe, [-5, -5, 0])
        self.change_in_r['medium'] = fuzz.trimf(self.change_in_r.universe, [-5, 0, 5])
        self.change_in_r['high'] = fuzz.trimf(self.change_in_r.universe, [0, 5, 5])
        self.rule1 = ctrl.Rule(self.dist['poor'], self.change_in_r['high'] )
        self.rule2 = ctrl.Rule(self.dist['average'], self.change_in_r['medium'])
        self.rule3 = ctrl.Rule(self.dist['good'], self.change_in_r['low'])
        self.tipping_ctrl = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3])
        self.tipping = ctrl.ControlSystemSimulation(self.tipping_ctrl)
        """

        # Custom membership functions can be built interactively with a familiar,
        # Pythonic API
        print(">> Starting circle (Thread)")
        rospy.spin()

    def fuzzy_calc(self):
        if self.rc.lidar.get_minDist() < 25:
           _yaw = self.rc.yaw
           if self.rc.yaw < 0:
              _yaw = self.rc.yaw + 2*pi
           self.centerCirc[0][0] = self.rc.pos[0][0] + cos((_yaw-self.rc.lidar.get_minRange_angles()[0]))*self.rc.lidar.get_minDist()
           self.centerCirc[1][0] = self.rc.pos[1][0] + sin((_yaw-self.rc.lidar.get_minRange_angles()[0]))*self.rc.lidar.get_minDist()
           print("lidar dist:\t"+ str(self.rc.lidar.get_minDist()))


    def calc_err(self):
        self.err = (self.rc.pos[0][0]-self.centerCirc[0][0])**2 + (self.rc.pos[1][0]-self.centerCirc[1][0])**2 -self.radius**2

    def update_grad_circle(self):
        self.grad = 2.0*(self.rc.pos[0:2]-self.centerCirc[0:2])

    def update_U(self):
        xt = self.rotM.dot(self.grad)
        xt_dot = self.rotM.dot(self.hess).dot(self.rc.vel[0:2])
        xt_norm = np.where( np.linalg.norm(xt) != 0., np.linalg.norm(xt), 0.00001 )
        self.vel_goal = self.s * xt / xt_norm
        xt_dot_xt = np.where( np.transpose(xt).dot(xt) != 0., np.transpose(xt).dot(xt)**(-3./2.), 1. )
        acc_d = xt_dot / xt_norm + xt * (-np.transpose(xt).dot(xt_dot)) * xt_dot_xt
        self.Ugvf = -self.c_pos * self.err * self.grad - self.c_vel * (self.rc.vel[0:2] - self.vel_goal) + self.c_acc*acc_d


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

        while self.state == "CIRCLE":
            self.pos_target.header.frame_id = "base_footprint"
            self.pos_target.header.stamp = rospy.Time.now()


            self.calc_err()
            self.update_grad_circle()
            self.update_U()

            print("self.Ugvf")
            self.pos_target.velocity.x = self.Ugvf[0][0] * 0.5
            self.pos_target.velocity.y = self.Ugvf[1][0] * 0.5
            v = (self.rc.pos[:2] - self.centerCirc)
            angle = atan2(v[1][0],v[0][0])
            if angle > pi :
                angle = -pi + (angle - pi)
            self.pos_target.yaw  = angle + pi
			# positioning
            self.rc.yaw = angle + pi



            self.fuzzy_calc()
            #self.local_vel_pub.publish(self.target)
            self.local_pos_pub.publish(self.pos_target)
            self.rate.sleep()

        '''
        print(">> Cicle has stopped")
        self.target.twist.linear.x = 0
        self.target.twist.linear.y = 0
        self.target.twist.linear.z = 0

        self.local_vel_pub.publish(self.target)
        '''

    """
    Stop
    """
    def stop(self,r):
        self.set_state("STOP")
        return {}

if __name__ == '__main__':
    Tj = traj_grp1()
