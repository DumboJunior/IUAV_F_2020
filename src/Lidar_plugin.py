# std import
import time
import numpy as np

# ros import
import rospy
import rospkg

# ros service messages
from sensor_msgs.msg import LaserScan

class Lidar_plugin(object):
	def __init__(self, ns):
		self.lidar_angle_dat = np.zeros(3)
		self.lidar_ranges = np.zeros(100)

		rospy.Subscriber(ns+"/scan", LaserScan, self.cb_lidar)

		# spin () simply keeps python from eciting until this node is stopped
		#rospy.spin()

	def cb_lidar(self, data):
		# set angle data {min, step, max} [rad]
		self.lidar_angle_dat[0] = data.angle_min
		self.lidar_angle_dat[1] = data.angle_increment
		self.lidar_angle_dat[2] = data.angle_max

		# set rages [m]
		self.lidar_ranges = data.ranges

	def get_minRange_angles(self):
		# returns angles for the min dist
		result = np.where(self.lidar_ranges == np.amin(self.lidar_ranges))
		min_dist_angles = result[0]*self.lidar_angle_dat[1]+self.lidar_angle_dat[0]
		return min_dist_angles

	def get_minDist(self):
		return np.amin(self.lidar_ranges)

