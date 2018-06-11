#!/usr/bin/env python

import math
import numpy
import rospy
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped

def point_in_poly(polygon, point):
	"""This is a method of determining if a given point is within a given polygon. 
	It assumes that the given point and polygon are both stamped instances.

	NOTE: If the point and polygon are in different frames then it wants to transform the point into the same frame.
	However, if this is being called very often the function transformPoint will slow performance greatly. 
	In a case like that, it is better to ensure the point and polgyon are in the same frame already."""
	if not isinstance(polygon, PolygonStamped):
		rospy.logerr("A stamped polygon was not passed into point_in_poly.  Aborting point_in_poly.")
		raise Exception("A stamped polygon was not passed into point_in_poly.  Aborting point_in_poly.")

	if not isinstance(point, PointStamped):
		rospy.logerr("A stamped point was not passed into point_in_poly.  Aborting point_in_poly.")
		raise Exception("A stamped point was not passed into point_in_poly.  Aborting point_in_poly.")

	n = len(polygon.polygon.points)

	result = False

	# Transforming the point into the same frame as the polygon if it is necessary
	if polygon.header.frame_id != point.header.frame_id:
		# point = tf_listener.transformPoint(polygon.header.frame_id, point)
		rospy.logerr("The polygon and point were not provided in the same frame.  Aborting point_in_poly.")
		raise Exception("The polygon and point were not provided in the same frame.  Aborting point_in_poly.")

	# Getting rid of stamped portions
	polygon = polygon.polygon
	point = point.point

	# Point in polygon algorithm
	p1x = polygon.points[0].x
	p1y = polygon.points[0].y
	for i in range(n):
		p2x = polygon.points[i].x
		p2y = polygon.points[i].y
		# print("p1y : %f p2y : %f" %(p1y,p2y))
		if point.y > min(p1y,p2y):
			# print "Y is above min"
			if point.y <= max(p1y,p2y):
				# print "Y is below max"
				if point.x <= max(p1x,p2x):
					# print "x below max"
					if p1y != p2y:
						xinters = (point.y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
					if p1x == p2x or point.x <= xinters:
						result = not result
		p1x,p1y = p2x,p2y
	return result