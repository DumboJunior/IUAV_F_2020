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
import pandas
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

        self.yaw = 0.
        self.tooclose = 0
        self.roll = 0.
        self.pitch = 0.

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
        x = data.pose.orientation.x
        y = data.pose.orientation.y
        z = data.pose.orientation.z
        w = data.pose.orientation.w

        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.where(t2>+1.0, +1.0, t2)
        #t2 = +1.0 if t2 > +1.0 else t2

        t2 = np.where(t2<-1.0, -1.0, t2)
        #t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        self.roll = roll
        self.pitch = pitch

class traj_grp1(object):
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.counture_points = []
        self.rcpos_X=[]
        self.rcpos_Y=[]
        self.circPos_X=[]
        self.circPos_Y=[]

        self.err = 0.0
        self.grad = np.array([[0.0],[0.0]])
        self.hess = np.array([[2.0,0.0],[0.0,2.0]])
        self.rc = rotocraft()
        self.vel_goal = np.array([[0.0],[0.0]])
        self.Ugvf = np.array([[0.0],[0.0],[15.0]])

        # Circle param
        self.centerCirc = np.array([[1.0],[5.0]])
        self.radius = 15.0

        # Regulation constants
        self.c_pos = 0.005#/self.radius
        self.c_vel= 3.0
        self.c_acc = 1.25

        self.s = 3.00
        self.count = 0
        # 90 deg Rot_Matrix
        self.rotM = np.array([[0.0, -1.0],[1.0, 0.0]])

        self.state = "INIT"

        self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

        self.local_pos_pub = rospy.Publisher(self.rc.ns+"/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        self.pos_target = PositionTarget()
        self.pos_target.coordinate_frame = 1
        # Ignore flags for pos and vel
        self.pos_target.type_mask =  1 + 2 + 8 + 16 + 32 + 2048

        self.const_vel = PositionTarget()
        self.const_vel.coordinate_frame = 1
        # Ignore flags for pos and vel
        self.const_vel.type_mask = 1 + 2 + 64 + 128 + 256 + 2048
        self.const_vel.velocity.x = 1
        self.const_vel.velocity.y = 1
        self.const_vel.position.z = 10
        self.const_vel.yaw=pi/4.

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

    def calculateDistance(self, point1 ,point2):
        dist = sqrt((point1[0] - point2[0] )**2 + (point1[1] -point2[1] )**2)
        return dist

    def averagePoint(self, points):
        average = [sum(x)/len(x) for x in zip(*points)]
        return average


    def furthestPointDist(self, points, center):
        max_dist = 0
        for point in points:
            new_point_dist = self.calculateDistance(point, center)

            if new_point_dist > max_dist :
                max_dist = new_point_dist
        return max_dist

    def avoidance(self):
        minDist = self.rc.lidar.get_minDist()
        angle = self.rc.lidar.get_minRange_angles()[0]
        if abs(self.rc.roll) < 0.2 and abs(self.rc.pitch) < 0.2 and minDist < 9. and self.rc.tooclose == 0:
            _yaw = self.rc.yaw
            x = self.rc.pos[0][0] + cos((_yaw + angle))*minDist
            y = self.rc.pos[1][0] + sin((_yaw + angle))*minDist
            print("angles:!\t",_yaw,angle)
            if x != 0.0 and y != 0.0:
                self.counture_points.append([x,y])
                average_point = self.averagePoint(self.counture_points)
                self.centerCirc[0][0] = average_point[0]
                self.centerCirc[1][0] = average_point[1]
                self.radius = self.furthestPointDist(self.counture_points, average_point) + 10.
                self.s = 0
                self.rc.tooclose = 1
                print("RC pos:")
                print(str(self.rc.pos))
                print("minDist:")
                print(str(minDist))
                print('radius:')
                print(self.radius)
                print('center:')
                print(self.centerCirc)
                print('points:')
                print(self.counture_points)
                print("Error:")
                print(str(self.err))
        elif abs(self.err) < 10.0 and self.rc.tooclose == 1: #self.rc.lidar.get_minDist() > 9.5 and self.rc.tooclose == 1:
            print('reset')
            self.s = 3
            self.rc.tooclose = 0
        else:
            print('Else radius:')
            print(self.radius)
            print('Else center:')
            print(self.centerCirc)
            print('Else points:')
            print(self.counture_points)
            print("ELSE ERROR:")
            print(str(self.err))
            print("RC pos:")
            print(str(self.rc.pos))
            print("minDist:")
            print(str(minDist))


    def calc_err(self):
        self.err = (self.rc.pos[0][0]-self.centerCirc[0][0])**2 + (self.rc.pos[1][0]-self.centerCirc[1][0])**2 - self.radius**2

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
            #print("roll")
            #print(self.rc.roll)
            #print("pitch")
            #print(self.rc.pitch)
            self.pos_target.acceleration_or_force.x = self.Ugvf[0][0] * 0.5
            self.pos_target.acceleration_or_force.y = self.Ugvf[1][0] * 0.5
            self.pos_target.position.z = 10
            self.rcpos_X.append(self.rc.pos[0][0])
            self.rcpos_Y.append(self.rc.pos[1][0])
            self.circPos_X.append(self.centerCirc[0][0])
            self.circPos_Y.append(self.centerCirc[1][0])

            v = (self.rc.pos[:2] - self.centerCirc)
            angle = atan2(v[1][0],v[0][0])
            if angle > pi :
                angle = -pi + (angle - pi)
            self.pos_target.yaw  = angle + pi
            if len(self.counture_points)<=0:
                self.rc.yaw = angle
            else:
                self.rc.yaw = angle + pi
            self.avoidance()

            if len(self.counture_points) <= 0:
                self.local_pos_pub.publish(self.const_vel)
            else:
                self.local_pos_pub.publish(self.pos_target)
            self.rate.sleep()

            rcPosCap = pandas.DataFrame(data={"x":self.rcpos_X, "y":self.rcpos_Y, "circ_x":self.circPos_X, "circ_y":self.circPos_Y})
            rcPosCap.to_csv("RC_avg_test.csv",sep=",",index=False)

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
