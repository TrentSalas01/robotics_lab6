#!/usr/bin/env python3
#import all necessary modules
import rospy
import math
import numpy as np
import cv2
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

#initiate XYZarray
xyz = XYZarray()

#function to get xyz from XYZarray
def get_xyz(XYZarray):
	global xyz
	xyz = XYZarray
	
#function to calculate center and radius
def sphere_params(xyz1):
	points = xyz1.points
	#calculations for B and A which is used to solve for P.
	B = [[i.x**2 + i.y**2 + i.z**2] for i in points if i.x < 1]
	A = [[2*i.x, 2*i.y, 2*i.z, 1] for i in points if i.x < 1]
	#solve for P
	P = np.linalg.lstsq(A, B, rcond = None)[0]
	#make P and numpy array
	P = np.array(P)
	#return the SPhereParams of the xc, yc, zc, and radius
	return SphereParams(P[0], P[1], P[2], math.sqrt(P[3] + P[0]**2 + P[1]**2 + P[2]**2))
	
	

if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('sphere_params', anonymous = True)
	# define a subscriber to get xyz of cropped ball
	img_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_xyz)
	# define a publisher to publish position
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)

	# set the loop frequency
	rate = rospy.Rate(10)
	
	x = 0
	
	fil_in = 0.0 # before the first reading
	fil_out = 5.5 # an initial guess before the first reading
	fil_gain = 0.05 # how much of the most recent input is included 
	while not rospy.is_shutdown():
		#get rid of empty lists
		if len(xyz.points) == 0:
			continue
		
		
		#publish node
		sphere_params1 = sphere_params(xyz)
		
		fil_in = sphere_params1.xc
		fil_out = fil_gain*fil_in + (1 - fil_gain)*fil_out
		sphere_params1.xc = fil_out
		
		fil_in = sphere_params1.yc
		fil_out = fil_gain*fil_in + (1 - fil_gain)*fil_out
		sphere_params1.yc = fil_out
		
		fil_in = sphere_params1.zc
		fil_out = fil_gain*fil_in + (1 - fil_gain)*fil_out
		sphere_params1.zc = fil_out
		
		print(sphere_params1.xc, sphere_params1.yc, sphere_params1.zc, sphere_params1.radius)
		sphere_pub.publish(sphere_params1)
		# pause until the next iteration			
		rate.sleep()
