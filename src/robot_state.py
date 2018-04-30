#!/usr/bin/env python
__author__ = "Chris Suarez"
__copyright__ = "Copyright 2018, The University of Texas at Austin, " \
                "Nuclear Robotics Group"
__credits__ = "Chris Suarez"
__license__ = "BSD"
__version__ = "0.0.1"
__maintainer__ = "Chris Suarez"
__email__ = "chriswsuarez@utexas.edu"
__status__ = "Production"
__doc__ = """This node is made to calculate the state of a robot including 
	height of a robot from base_link to the highest measured link and the
	footprint of a robot displayed as a polygon. 
	It publishes the x,y,z coordinates of only the highest robot point
	and a polygon for the footprint given all the links"""

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

	# Adjustable settings
	pub_rate = 1 # hz
	ref_link = '/base_footprint' # the reference frame which will be considered as the origin of the robot on the ground
	halo_radius = 0.25 # m
	halo_density = 6 # no. of points in the halo buffer
	hgt_pub_top = '~robot_height' # topic name to publish the robot height
	ftprnt_pub_top = '~robot_footprint' # topic name to publish the robot footprint
	frames_to_ignore = ['map', 'odom', 'left_ur5_tool0_controller', 'right_ur5_tool0_controller'] # This is a list of frames in the TF tree that you want to leave out of the robot footprint. NOTE: no leading slash here

	#----------------------------------------------
	# Protected Class Variables
	_highest_point = Point32()
	_highest_link = 'Not init'
	_robot_footprint = PolygonStamped()
	_robot_footprint.header.frame_id = ref_link

	def __init__(self):
		self._pub_height = rospy.Publisher(self.hgt_pub_top, Point32, queue_size=1)
		self._pub_footprint = rospy.Publisher(self.ftprnt_pub_top, Polygon, queue_size=1)
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

		pub_rate = rospy.Rate(self.pub_rate)
		pub_rate.sleep()



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

		p_rate = rospy.Rate(_RobotState.pub_rate)
		# This is the ros spinner
		while not rospy.is_shutdown():
			_RobotState.analyze_links()
			p_rate.sleep()
	except rospy.ROSInterruptException: 
		pass