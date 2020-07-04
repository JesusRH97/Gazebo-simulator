#!/usr/bin/env python

from road import Road
from curved_road import Curved_Road
from straight_road import Straight_Road 
from rospkg import RosPack
from numpy import * 
import math
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
import rospy

"""
	The following class implements the attributes and methods
	for creating a circuit
"""

class Circuit(object):
	
	def __init__(self):
		
		self.roads = []
		self.points = []


	def insert_road(self, type_of_road, dimension, x, y, z, roll, pitch, yaw):

		if(type_of_road == "curva"):
			curve = Curved_Road(type_of_road, x, y, z, roll, pitch, yaw, dimension)
			self.roads.append(curve)

		else:
			straight_road = Straight_Road(type_of_road, x, y, z, roll, pitch, yaw, dimension)
			self.roads.append(straight_road)



	def get_road(self, position):
		
		return self.roads[position]



	def add_points(self):

		for i in range(len(self.roads)):
		
			#The first point of the road
			A = self.roads[i].points[0]
			#The last point of the road
			B = self.roads[i].points[len(self.roads[i].points)-1]

			#We obtain the center of the following road

			if i == len(self.roads)-1:

				"""
				for j in range(len(self.roads[i].points)):
					self.points.append(self.roads[i].points[j])
				"""
				
				last_point = self.points[len(self.points)-1]

				#We calculate the distance of both 
				#extremes to the center of the 
				#following road

				distance_A = self.calculate_distance(A, last_point)
				distance_B = self.calculate_distance(B, last_point)

				#Depending on the distance between the two extremes of the road to
				#the center of the following road, we will "append" the points
				#in one way or another

				if distance_A < distance_B:
					for j in range(len(self.roads[i].points)):
						self.points.append(self.roads[i].points[j])

				else:
					for j in range(len(self.roads[i].points)-1, 0, -1):
						self.points.append(self.roads[i].points[j])
				
				


			else:

				next_point = self.roads[i+1].points[0]

				#We calculate the distance of both 
				#extremes to the center of the 
				#following road

				distance_A = self.calculate_distance(A, next_point)
				distance_B = self.calculate_distance(B, next_point)

				#Depending on the distance between the two extremes of the road to
				#the center of the following road, we will "append" the points
				#in one way or another

				if distance_A > distance_B:

					for j in range(len(self.roads[i].points)):
						self.points.append(self.roads[i].points[j])

				else:

					for j in range(len(self.roads[i].points)-1, 0, -1):
						self.points.append(self.roads[i].points[j])






	def calculate_distance(self, vector1, vector2):

		distance = sqrt(pow(vector1[0] - vector2[0], 2) + pow(vector1[1]
		 - vector2[1], 2) + pow(vector1[2] - vector2[2], 2))

		return distance



	def read_input(self):

		dato1=[]
		dato2=[]
		dato3=[]
		dato4=[]
		dato5=[]
		dato6=[]
		dato7=[]
		dato8=[]


		rp = RosPack()

		path = rp.get_path('Gazebo-simulator')

		f = open (path+'/src/classes/ExteriorCircuit/input.txt','r')
		
		linea = f.readlines()


		for i in range(len(linea)):

		   dato1=linea[i].find('\t')
		   dato2=linea[i].find('\t',dato1+1)
		   dato3=linea[i].find('\t',dato2+1)
		   dato4=linea[i].find('\t',dato3+1)
		   dato5=linea[i].find('\t',dato4+1)
		   dato6=linea[i].find('\t',dato5+1)
		   dato7=linea[i].find('\t',dato6+1)
		   dato8=linea[i].find('\t',dato7+1)

		   valor1=(linea[i][0:dato1])
		   valor2=float(linea[i][dato1+1:dato2])
		   valor3=float(linea[i][dato2+1:dato3])
		   valor4=float(linea[i][dato3+1:dato4])
		   valor5=float(linea[i][dato4+1:dato5])
		   valor6=float(linea[i][dato5+1:dato6])
		   valor7=float(linea[i][dato6+1:dato7])
		   valor8=float(linea[i][dato7+1:dato8])

		   self.insert_road(valor1, valor2, valor3, valor4, valor5, valor6, valor7, valor8)




		f.close()




	def run(self):

		#Read the features of every road
		self.read_input()

		#Calculate the path of every road
		for i in range(len(self.roads)):
			self.roads[i].calculate_path()
			
		#Then, we start to add the points previously
		#calculated to the list of points of the circuit
		self.add_points()

