#!/usr/bin/env python

from road import Road
from numpy import * 
import math
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
import rospy


"""
	The following class implements the attributes and methods
	for creating a straight road
"""


class Straight_Road(Road):

	"""
		Constructor of the class. It calls its father through the super() method
	"""

	def __init__(self, type_of_road, x, y, z, roll, pitch, yaw, dimension):

		super(Straight_Road, self).__init__(type_of_road, x, y, z, roll, pitch, yaw)
		
		self.lenght = dimension
		self.corners = []


	"""
		This method is the most important one. It calculates 
		all the points which form the road. The points are
	    calculated in X, Y and Z coordinates
	"""

	def calculate_path(self):


		alpha = self.yaw

		m = tan(alpha)

		x = self.x + (10*sin(alpha))
		y = self.y - (10*cos(alpha))

		n = y - (m*x)

		start = x - abs(((self.lenght/2)*cos(alpha)))
		end = x + abs(((self.lenght/2)*cos(alpha)))

		self.corners.append(start)
		self.corners.append(end)


		while self.corners[0]<=self.corners[1]:

			y = (m*self.corners[0]) + n
			self.points.append([self.corners[0], y, 0])
			self.corners[0] = self.corners[0] + abs((0.1*cos(alpha)))


	"""
		This method returns the points of the road
	"""

	def get_path(self):
		
		return self.points



