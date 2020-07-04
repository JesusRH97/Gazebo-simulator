#!/usr/bin/env python

"""
	Class road. The following class implements the attributes and
	methods for creating just a road.
"""

class Road(object):

	def __init__(self, type_of_road, x, y, z, roll, pitch, yaw):

		self.type_of_road = type_of_road
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.points = []

	"""
		Abstract method. It will be developed
		in curved road and straight road classes.
		Basically it calculates the points of each road.
	"""

	def calculate_path(self):
		pass


	"""
		Abstract method. It will be developed
		in curved_road and straight road classes.
		Basically it returns the points of each road.
	"""

	def get_path(self):
		pass


	def get_x(self):
		return self.x

	def get_y(self):
		return self.y

	def get_z(self):
		return self.z

	def get_roll(self):
		return self.roll

	def get_pitch(self):
		return self.pitch

	def get_yaw(self):
		return self.yaw

	def get_type_of_road(self):
		return self.type_of_road

	def print_info(self):

		print(self.type_of_road+" "+str(self.x)+" "+str(self.y)+" "
			+str(self.z)+" "+str(self.roll)+" "+str(self.pitch)+" "+str(self.yaw))

	def print_points(self):

		print("This road contains a total of "+str(len(self.points))+" points")



		
		
		
