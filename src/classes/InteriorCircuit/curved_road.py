#!/usr/bin/env python

from road import Road
from numpy import * 
import math
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
import rospy

"""
	The following class implements the attributes and methods
	for creating a curved road
"""

class Curved_Road(Road):

	"""
		Constructor of the class. It calls its father through the super() method
	"""

	def __init__(self, type_of_road, x, y, z, roll, pitch, yaw, dimension):

		super(Curved_Road, self).__init__(type_of_road, x, y, z, roll, pitch, yaw)
		
		self.radius = dimension
		self.center = []
		self.corners = []

		if(dimension == 100):
			self.side = 104
		else:
			self.side = 55


	"""
		This method calculates the diagonal of the square in which the
		curve is drawed
	"""

	def calculate_diagonal(self):

		return(sqrt(pow(self.side/2,2) + pow(self.side/2,2)))



	"""
		This method is the most important one. It calculates 
		all the points which form the road. The points are
	    calculated in X, Y and Z coordinates
	"""

	def calculate_path(self):

		

		self.diagonal = self.calculate_diagonal()

		alpha = self.yaw


		x = self.x + (self.diagonal*cos((pi/4) + alpha))
		y = self.y + (self.diagonal*sin((pi/4) + alpha))


		Ox = x - (self.side*cos(alpha))
		Oy = y - (self.side*sin(alpha))
	

		self.center.append(Ox)
		self.center.append(Oy)

		angle = 0.0

		while angle<=pi/2:

			eje_x = self.center[0] + (49*sin(alpha + angle))
			eje_y = self.center[1] - (49*cos(alpha + angle))
			self.points.append([eje_x, eje_y, 0])
			angle += 0.001


	"""
		This method returns the points of the road
	"""

	def get_path(self):
		
		return self.points




		
