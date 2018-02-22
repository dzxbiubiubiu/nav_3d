#!/usr/bin/env python

import rospy
import numpy
import tf
from scipy import spatial
from geometry_msgs.msg import Point32, PolygonStamped, Polygon


class robot_state:
	""" This class is made to calculate the state of a robot including 
	height of a robot from base_link to the highest measured link and the
	footprint of a robot displayed as a polygon. 
	It publishes the x,y,z coordinates of only the highest robot point
	and a polygon for the footprint given all the links."""

	# Adjustable settings
	pub_rate_ = 1 # hz
	ref_link_ = '/base_footprint' # the reference frame which will be considered as the origin of the robot on the ground
	halo_radius_ = 0.2 # m
	halo_density_ = 10 # no. of points in the halo buffer
	hgt_pup_top_ = '/nav_3d/robot_height' # topic name to publish the robot height
	ftprnt_pub_top_ = '/nav_3d/robot_footprint' # topic name to publish the robot footprint
	frames_to_ignore_ = ['map', 'odom', 'left_ur5_tool0_controller'] # This is a list of frames in the TF tree that you want to leave out of the robot footprint. NOTE: no leading slash here

	#----------------------------------------------
	highest_point_ = Point32()
	highest_link_ = 'Not init'
	robot_footprint_ = PolygonStamped()
	robot_footprint_.header.frame_id = ref_link_

	send_msg_ = False

	def __init__(self):
		pub_height = rospy.Publisher(self.hgt_pup_top_, Point32, queue_size=1)
		pub_footprint = rospy.Publisher(self.ftprnt_pub_top_, Polygon, queue_size=1)

		pub_rate = rospy.Rate(self.pub_rate_)
		while not rospy.is_shutdown():
			self.analyze_links()
			if self.send_msg_:
				pub_height.publish(self.highest_point_)
				pub_footprint.publish(self.robot_footprint_.polygon)
			pub_rate.sleep()

	def analyze_links(self):
		"""This function just iterates through every link in the robot and
		saves their XYZ positions for use by the find_height and footprint
		functions.  It bases the XYZ position using the reference link as
		the origin.  The reference link can be changed ref_link_ class variable."""

		# Get all the names for each link in the system
		links = tf_listener.getFrameStrings()

		link_points = []

		prev_high_link = self.highest_link_
		prev_high_point = self.highest_point_.z
		self.highest_point_ = Point32()
		self.highest_link_ = 'Not init'

		# Check the position of each link for the polygon footprint and check the height of each
		for frame in links:
			# Need to build build in frames not designated in the frames_to_ignore 
			if not frame in self.frames_to_ignore_:
				try:
					transform = tf_listener.lookupTransform(self.ref_link_, frame, rospy.Time())
	
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
		if not prev_high_link == self.highest_link_:
			rospy.loginfo("Highest link is now: %s at height %f meters above %s", self.highest_link_, self.highest_point_.z, self.ref_link_)
		elif prev_high_link == self.highest_link_ and not round(prev_high_point,2) == round(self.highest_point_.z,2):
			rospy.loginfo("%s is still the highest link but has moved to: %f meters above %s", self.highest_link_, self.highest_point_.z, self.ref_link_)
		else:
			pass

		self.robot_footprint_.polygon = self.find_footprint(link_points)
		self.robot_footprint_.header.stamp = rospy.Time.now()
		self.send_msg_ = True


	def check_height(self, position, frame):		
		if position[2] > self.highest_point_.z:
			self.highest_link_ = frame
			self.highest_point_.x = position[0]
			self.highest_point_.y = position[1]
			self.highest_point_.z = position[2]

	def find_footprint(self, points):
		"""This function will take in an numpy.array of points in 2D and create
		a convex polygon that encapsilates every point provided.\
		Arguments to pass in:
		points = 2D array of points numpy.array([:,:])
		Returns a Polygon"""
		footprint = Polygon()

		if self.halo_density_:
			buffed_points = self.halo_buffer(points, self.halo_radius_, self.halo_density_)
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
	rospy.init_node('nav_3d/robot_state')
	tf_listener = tf.TransformListener()
	try:
		robot_state = robot_state()
	except rospy.ROSInterruptException: 
		pass
