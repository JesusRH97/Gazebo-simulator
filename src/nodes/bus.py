#!/usr/bin/env python

import math
import rospy
import sys
import os
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler, euler_from_quaternion

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +"/../classes/InteriorCircuit/")

try:
    from circuit import Circuit
except ImportError:
    raise


def lane(position_x, position_y, angle, lane):
	
	if(lane == "right"):
		x = position_x + (2.5*math.sin(angle - 1.57))
		y = position_y - (2.5*math.cos(angle - 1.57))
		z = 0.11

	else:
		x = position_x - (2.5*math.sin(angle - 1.57))
		y = position_y + (2.5*math.cos(angle - 1.57))
		z = 0.11

	return x, y, z


def calculate_position_and_orientation(nodo, position, previous_position):

	X =  position[0] - previous_position[0]
	Y = position[1] - previous_position[1]
	angle = (math.atan2(Y,X) + 1.57)

	quaternion = quaternion_from_euler(0.0, 0.0, angle)

	nodo.pose.orientation.x = quaternion[0]
	nodo.pose.orientation.y = quaternion[1]
	nodo.pose.orientation.z = quaternion[2]
	nodo.pose.orientation.w = quaternion[3]

	nodo.pose.position.x, nodo.pose.position.y, nodo.pose.position.z = lane(position[0], position[1], angle, "left")

	return nodo



def publish():

	rospy.init_node("node1", anonymous=False)
	pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
	rate = rospy.Rate(200)

	nodo = ModelState()
	nodo.model_name = "bus"

	circuit = Circuit()
	circuit.run()


	for i in range(len(circuit.points)):

		position = circuit.points[i]
		previous_position = circuit.points[i-1]

		nodo = calculate_position_and_orientation(nodo, position, previous_position)
		
		pub.publish(nodo)
		rate.sleep()	



	while not rospy.is_shutdown():
		
		for i in range(len(circuit.points)):

			position = circuit.points[i]
			previous_position = circuit.points[i-1]

			nodo = calculate_position_and_orientation(nodo, position, previous_position)
			
			pub.publish(nodo)
			rate.sleep()		




if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass