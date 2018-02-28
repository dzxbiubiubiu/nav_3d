#!/usr/bin/env python

import rospy
import numpy
import tf
from scipy import spatial
from geometry_msgs.msg import Point32, PolygonStamped, Polygon
from sensor_msgs.msg import LaserScan, PointCloud2


class MapMaintainer:
	""" poop """

	# Adjustable settings
	pub_rate = 1 # hz
	ground_link = '/base_footprint' # the reference frame which will be considered as the origin of the robot on the ground
	scan_link = '/hokuyo_lidar' # the link from which you want the cone to be cast to the ground_link
	cone_density = 10 # no. of points displayed on the ground from the cone
	cone_pub_top = '/nav_3d/shadow_cone' # topic name to publish the robot height
	map_pub_top = 'nav_3d/costmap' # topic name to publish the aggregated costmap

	#----------------------------------------------
	# Protected Class Variables
	_cone_footprint = PolygonStamped()

	_send_msg = False

	def __init__(self):
		pub_cone = rospy.Publisher(self.cone_pub_top, PolygonStamped, queue_size=1)
		pub_map = rospy.Publisher(self.map_pub_top, , queue_size=1)

		pub_rate = rospy.Rate(self.pub_rate)
		while not rospy.is_shutdown():
			self.analyze_links()
			if self._send_msg:
				pub_height.publish(self._highest_point)
				pub_footprint.publish(self._robot_footprint.polygon)
			pub_rate.sleep()

	def analyze_links(self):
		"""This function just iterates through every link in the robot and
		saves their XYZ positions for use by the find_height and footprint
		functions.  It bases the XYZ position using the reference link as
		the origin.  The reference link can be changed ref_link class variable."""

		# Get all the names for each link in the system

		self._send_msg = True


if __name__=='__main__':
	rospy.init_node('map_maintainer')
	tf_listener = tf.TransformListener()
	try:
		_MapMaintainer = MapMaintainer()
	except rospy.ROSInterruptException: 
		pass
