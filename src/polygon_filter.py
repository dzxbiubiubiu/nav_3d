#!/usr/bin/env python

import rospy
import numpy
import math
from tf import TransformerROS, TransformListener
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped
from laser_geometry import LaserProjection

class polygon_filter:
	# Change these variables to adjust the filter
	scan_sub_topic = "/scan360" # "/SICK_LMS291/scan" # "/scan360"
	poly_sub_topic =   "/move_base/local_costmap/footprint" # "test_poly_node/polygon"
	pub_topic = "/polygon_filter/scan360"
	# The filter works by removing any laserscan point 
	# that is within a given polygon

	current_poly = PolygonStamped()
	poly_init = False
	cloud = PointCloud2()
	laser_projector = LaserProjection()

	def __init__(self):
		self.send_msg_ = False
		rospy.Subscriber(self.scan_sub_topic, LaserScan, self.filter_scan)
		rospy.Subscriber(self.poly_sub_topic, PolygonStamped, self.update_poly)
		pub = rospy.Publisher(self.pub_topic, LaserScan, queue_size=10)
		while not rospy.is_shutdown():
			if self.send_msg_:
				pub.publish(self.new_scan)
				self.send_msg_ = False

	def filter_scan(self, scan):
		# A callback to filter the input scan and output a new filtered scan

		# If there is no polygon initialized then we will just spit out the scan received
		if self.poly_init:
			self.new_scan = scan
			self.new_scan.range_min = 0.1
			self.new_scan.range_max = 30.0
			self.new_scan.ranges = list(scan.ranges)

			self.cloud = self.laser_projector.projectLaser(scan)
			# gen = pc2.read_points(self.cloud, skip_nans = True, field_names=("x", "y", "z"))
			# self.xyz_generator = gen

			laser_point = PointStamped()
			laser_point.header = scan.header

			# If the polygon and the points being read are in different frames we'll transform the polygon
			if self.current_poly.header.frame_id != laser_point.header.frame_id:
				# print("Tranforming the polygon")
				# tf_listener.waitForTransform(point.header.frame_id, polygon.header.frame_id, rospy.Time(), rospy.Duration(0.10))
				i = 0
				temp_poly_point = PointStamped()
				for v in self.current_poly.polygon.points:
					temp_poly_point.header = self.current_poly.header
					temp_poly_point.point = v # self.current_poly.polygon.points[i]
					temp_poly_point = tf_listener.transformPoint(laser_point.header.frame_id, temp_poly_point)
					self.current_poly.polygon.points[i] = temp_poly_point.point
					i = i + 1
				self.current_poly.header.frame_id = laser_point.header.frame_id


			i = 0
			for p in pc2.read_points(self.cloud, field_names = ("x", "y", "z"), skip_nans=True):
				# print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
				laser_point.point.x = p[0]
				laser_point.point.y = p[1]
				laser_point.point.z = p[2]

				# This for loop does not take into account inf points in the original scan. Need to count those too
				while scan.ranges[i] == numpy.inf:
					i = i + 1

				# Check to see in the laser x and y are within the polygon
				in_poly = point_in_poly(self.current_poly, laser_point)

				# If the point is within the polygon we should throw it out (give it a value of inf)
				if in_poly: self.new_scan.ranges[i] = numpy.inf
				i = i + 1

		else:
			self.new_scan = scan
		self.send_msg_ = True

	def update_poly(self, poly_stamped):
		"""Only updates the current_poly to the newly published polygon"""
		self.current_poly = poly_stamped
		self.poly_init = True

if __name__=='__main__':
	tf_listener = TransformListener()
	rospy.init_node('nav_3d/polygon_filter')
	try:
		polygon_filter = polygon_filter()
	except rospy.ROSInterruptException: 
		pass
