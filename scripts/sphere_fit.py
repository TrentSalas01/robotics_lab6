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
	
	xcfil_out = -0.013 # an initial guess before the first reading
	xcfil_gain = 0.05/2 # how much of the most recent input is included 
	
	ycfil_out = -0.02 # an initial guess before the first reading
	ycfil_gain = 0.05/2 # how much of the most recent input is included 

	zcfil_out = 0.45 # an initial guess before the first reading
	zcfil_gain = 0.05/2 # how much of the most recent input is included 
	
	radiusfil_out = 0.035 # an initial guess before the first reading
	radiusfil_gain = 0.05/2 # how much of the most recent input is included 
	
	while not rospy.is_shutdown():
		#get rid of empty lists
		if len(xyz.points) == 0:
			continue
			
		#publish node
		sphere_params1 = sphere_params(xyz)
		
		# math to calculate the noise out (XC)
		xcfil_in = sphere_params1.xc
		xcfil_out = xcfil_gain*xcfil_in + (1 - xcfil_gain)*xcfil_out
		sphere_params1.xc = xcfil_out
		
		# math to calculate the noise out (YC)
		ycfil_in = sphere_params1.yc
		ycfil_out = ycfil_gain*ycfil_in + (1 - ycfil_gain)*ycfil_out
		sphere_params1.yc = ycfil_out
		
		# math to calculate the noise out (ZC)
		zcfil_in = sphere_params1.zc
		zcfil_out = zcfil_gain*zcfil_in + (1 - zcfil_gain)*zcfil_out
		sphere_params1.zc = zcfil_out
		
		# math to calculate the noise out (RADIUS)
		radiusfil_in = sphere_params1.zc
		radiusfil_out = radiusfil_gain*radiusfil_in + (1 - radiusfil_gain)*radiusfil_out
		sphere_params1.zc = radiusfil_out
		
		# print for test
		print(sphere_params1.xc, sphere_params1.yc, sphere_params1.zc, sphere_params1.radius)
		# publish
		sphere_pub.publish(sphere_params1)
		# pause until the next iteration			
		rate.sleep()
