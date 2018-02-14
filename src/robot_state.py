#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, PolygonStamped, Polygon
# from std_msgs.msg import Float32
# from tf import TransformListener
import tf
from scipy import spatial

class robot_state:
	""" This class is made to calculate the state of a robot including 
	height of a robot from base_link to the highest measured link and the
	footprint of a robot displayed as a polygon. 
	It publishes the x,y,z coordinates of only the highest robot point
	and a polygon for the footprint given all the links."""

	# Adjustable settings
	pub_rate_ = 1 # hz
	ref_link_ = '/base_footprint'
	#----------------------------------------------

	highest_point_ = Point32()
	highest_link_ = 'Not init'
	robot_footprint_ = PolygonStamped()
	robot_footprint_.header.frame_id = ref_link_

	send_msg_ = False

	def __init__(self):
		pub_height = rospy.Publisher('/robot_height', Point32, queue_size=1)
		pub_footprint = rospy.Publisher('/robot_footprint', PolygonStamped, queue_size=1)

		pub_rate = rospy.Rate(self.pub_rate_)
		while not rospy.is_shutdown():
			self.analyze_links()
			if self.send_msg_:
				pub_height.publish(self.highest_point_)
				pub_footprint.publish(self.robot_footprint_)
			pub_rate.sleep()

	def analyze_links(self):
		"""This function just iterates through every link in the robot and
		saves their XYZ positions for use by the find_height and footprint
		functions.  It bases the XYZ position using the reference link as
		the origin.  The reference link can be changed ref_link_ class variable."""

		# Get all the names for each link in the system
		links = tf_listener.getFrameStrings()

		link_points = []
		# Check the position of each link and check to find the highest link
		i = 0
		for frame in links:
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
		
		self.robot_footprint_.polygon = self.find_footprint(link_points)
		self.robot_footprint_.header.stamp = rospy.Time.now()
		self.send_msg_ = True


	def check_height(self, position, frame):
		prev_high_link = self.highest_link_
		prev_high_point = self.highest_point_.z
		
		if position[2] > self.highest_point_.z:
			self.highest_link_ = frame
			self.highest_point_.x = position[0]
			self.highest_point_.y = position[1]
			self.highest_point_.z = position[2]

		if not prev_high_link == self.highest_link_:
			rospy.loginfo("Highest link is now: %s at height %f meters above base_link", self.highest_link_, self.highest_point_.z)
		elif prev_high_link == self.highest_link_ and not round(prev_high_point,2) == round(self.highest_point_.z,2):
			rospy.loginfo("%s is still the highest link but has moved to: %f meters above base link", self.highest_link_, self.highest_point_.z)
		else:
			pass
		
	def find_footprint(self, points):
		footprint = Polygon()

		if not points:
			pass
		else:
			hull = spatial.ConvexHull(points[:][:])
			for vertex in hull.vertices:
				point = Point32()
				point.x = points[vertex][0]
				point.y = points[vertex][1]
				point.z = 0.0
				footprint.points.append(point)
		return footprint

if __name__=='__main__':
	rospy.init_node('robot_state')
	tf_listener = tf.TransformListener()
	try:
		robot_state = robot_state()
	except rospy.ROSInterruptException: 
		pass
