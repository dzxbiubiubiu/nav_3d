#!/usr/bin/env python

#///////////////////////////////////////////////////////////////////////////////
#//      Title     : robot_state.py
#//      Project   : nav_3d
#//      Created   : 1/23/2018
#//      Author    : Chris Suarez
#//      Platforms : Ubuntu 64-bit
#//      Copyright : Copyright The University of Texas at Austin, 2014-2017. All rights reserved.
#//                 
#//          All files within this directory are subject to the following, unless an alternative
#//          license is explicitly included within the text of each file.
#//
#//          This software and documentation constitute an unpublished work
#//          and contain valuable trade secrets and proprietary information
#//          belonging to the University. None of the foregoing material may be
#//          copied or duplicated or disclosed without the express, written
#//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
#//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
#//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
#//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
#//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
#//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
#//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
#//          University be liable for incidental, special, indirect, direct or
#//          consequential damages or loss of profits, interruption of business,
#//          or related expenses which may arise from use of software or documentation,
#//          including but not limited to those resulting from defects in software
#//          and/or documentation, or loss or inaccuracy of data of any kind.
#//
#///////////////////////////////////////////////////////////////////////////////

import rospy
import numpy
import tf
from scipy import spatial
from geometry_msgs.msg import Point32, PolygonStamped, Polygon


class RobotState:
	""" This class is made to calculate the state of a robot including 
	height of a robot from base_link to the highest measured link and the
	footprint of a robot displayed as a polygon. 
	It publishes the x,y,z coordinates of only the highest robot point
	and a polygon for the footprint given all the links."""

	def __init__(self):
		# Adjustable params from robot_state.yaml
		self.ref_link = rospy.get_param("nav_3d/robot_state/robot_footprint_frame", "/base_footprint")
		self.halo_radius = rospy.get_param("nav_3d/robot_state/halo_radius", 0.25)
		self.halo_density = rospy.get_param("nav_3d/robot_state/halo_density", 5)
		self.hgt_pub_top = rospy.get_param("nav_3d/robot_state/height_pub_topic", "~robot_height")
		self.ftprnt_pub_top = rospy.get_param("nav_3d/robot_state/footprint_pub_topic", "~robot_footprint")
		self.frames_to_ignore = rospy.get_param("nav_3d/robot_state/frames_to_ignore", ["map", "odom"])
		self.loop_rate = rospy.get_param("nav_3d/robot_state/loop_rate", 1)

		# Protected Class Variables
		self._highest_point = Point32()
		self._highest_link = 'Not init'
		self._robot_footprint = PolygonStamped()
		self._robot_footprint.header.frame_id = self.ref_link

		# Publishers, subscribers and tf listener
		self._pub_height = rospy.Publisher(self.hgt_pub_top, Point32, queue_size=1)
		self._pub_footprint = rospy.Publisher(self.ftprnt_pub_top, Polygon, queue_size=1)
		self._pub_footprint_stamped = rospy.Publisher( (self.ftprnt_pub_top + "_stamped"), PolygonStamped, queue_size=1)
		self._tf_listener = tf.TransformListener()

	def analyze_links(self):
		"""This function just iterates through every link in the robot and
		saves their XYZ positions for use by the find_height and footprint
		functions.  It bases the XYZ position using the reference link as
		the origin.  The reference link can be changed ref_link class variable."""

		# Get all the names for each link in the system
		links = self._tf_listener.getFrameStrings()

		link_points = []

		prev_high_link = self._highest_link
		prev_high_point = self._highest_point.z
		self._highest_point = Point32()
		self._highest_link = 'Not init'

		# Check the position of each link for the polygon footprint and check the height of each
		for frame in links:
			# Need to build build in frames not designated in the frames_to_ignore 
			if not frame in self.frames_to_ignore:
				try:
					transform = self._tf_listener.lookupTransform(self.ref_link, frame, rospy.Time())
	
					# The position array is the first section of the tuple transform[0] = [x,y,z]
					position = transform[0]
					self.check_height(position, frame)
	
					# Building the XY list of each link to send to find_footprint
					position.pop()
					link_points.append(position)
	
				except tf.Exception:
					rospy.logerr_throttle(5, 'TF has thrown an exception.  Will retry the TF call')
		
		# Converting the list of link points to an array so it's easier to work with
		link_points = numpy.array(link_points)

		# After all of the frames have been analyzed we now determine the highest link
		if not prev_high_link == self._highest_link:
			rospy.loginfo("Highest link is now: %s at height %f meters above %s", self._highest_link, self._highest_point.z, self.ref_link)
		elif prev_high_link == self._highest_link and not round(prev_high_point,2) == round(self._highest_point.z,2):
			rospy.loginfo("%s is still the highest link but has moved to: %f meters above %s", self._highest_link, self._highest_point.z, self.ref_link)
		else:
			pass

		self._robot_footprint.polygon = self.find_footprint(link_points)
		self._robot_footprint.header.stamp = rospy.Time.now()
		
		self._pub_height.publish(self._highest_point)
		self._pub_footprint.publish(self._robot_footprint.polygon)
		self._pub_footprint_stamped.publish(self._robot_footprint)

		loop_rate = rospy.Rate(self.loop_rate)
		loop_rate.sleep()



	def check_height(self, position, frame):		
		if position[2] > self._highest_point.z:
			self._highest_link = frame
			self._highest_point.x = position[0]
			self._highest_point.y = position[1]
			self._highest_point.z = position[2]

	def find_footprint(self, points):
		"""This function will take in an numpy.array of points in 2D and create
		a convex polygon that encapsilates every point provided.\
		Arguments to pass in:
		points = 2D array of points numpy.array([:,:])
		Returns a Polygon"""
		footprint = Polygon()

		if self.halo_density:
			buffed_points = self.halo_buffer(points, self.halo_radius, self.halo_density)
		else:
			buffed_points = points

		if not points.size:
			pass
		else:
			hull = spatial.ConvexHull(buffed_points)
			for vertex in hull.vertices:
				point = Point32()
				point.x = buffed_points[vertex, 0]
				point.y = buffed_points[vertex, 1]
				point.z = 0.0
				footprint.points.append(point)
		return footprint

	def halo_buffer(self, points, radius, density):
		"""This function is designed to take in a 2D array points and 
		create add a radius of new points with the center of each of the
		original points.  This is useful when dealing with the footprint
		of a robot to create a better and safer robot footprint.
		Arguments to pass in:
		points = 2D array of points numpy.array([:,:])
		radius = buffer distance from the center of each point
		density = number of points to form the circle around each point
		Returns an 2D array of points"""
		new_points = list()
		theta_increment = 2 * 3.1416 / density

		for point in points:
			current_theta = 0
			for i in list(range(0, density)):
				x_new = point[0] + radius * numpy.cos(current_theta + theta_increment)
				y_new = point[1] + radius * numpy.sin(current_theta + theta_increment)
				new_points.append([x_new, y_new])

				current_theta += theta_increment

		new_points = numpy.array(new_points)
		return new_points


if __name__=='__main__':
	rospy.init_node('robot_state')

	try:
		_RobotState = RobotState()

		l_rate = rospy.Rate(_RobotState.loop_rate)
		# This is the ros spinner
		while not rospy.is_shutdown():
			_RobotState.analyze_links()
			l_rate.sleep()
	except rospy.ROSInterruptException: 
		pass
