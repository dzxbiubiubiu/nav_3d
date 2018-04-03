#!/usr/bin/env python

import rospy
import numpy
import tf
import math
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from point_in_poly import point_in_poly


class LiveNav:
	"""This is designed to dynamically create a obstacle laserscan over time via a build up of XYZ point cloud data."""
	
	# Change these variables to adjust the filter
	scan_sub_topic = "/laser_stitcher/planar_cloud"
	poly_sub_topic =   "/move_base/local_costmap/footprint" # "test_poly_node/polygon"
	height_sub_topic = "/nav_3d/robot_height"
	pub_topic = "~scan360"
	robot_height_default = 1.0 # m
	floor_range = 0.15 # m
	res = 3 # decimal points
	min_obj_range = 0.1 # Min obstacle range to be built into the costmap
	pub_rate = 100 # Hz
	
	#----------------------------------------------
	# Protected class varaibles
	_scan_to_publish = LaserScan()
	_scan_from_cloud = LaserScan()
	_laser_projector = LaserProjection()

	_current_poly = PolygonStamped()
	_poly_init = False

	_current_robot_height = Point32()
	# Initialize to the default robot height
	_current_robot_height.z = robot_height_default
	_robot_height_init = False

	_ang_dis_list = list()
	_angles_checked = list()
	_iteration_stamp = 0

	def __init__(self):
		self._pub = rospy.Publisher(self.pub_topic, LaserScan, queue_size=1)
		self._tf_listener = tf.TransformListener()
		rospy.Subscriber(self.scan_sub_topic, PointCloud2, self.scan_builder)
		rospy.Subscriber(self.poly_sub_topic, PolygonStamped, self.update_poly)
		rospy.Subscriber(self.height_sub_topic, Point32, self.update_height)



	def scan_builder(self, planar_cloud):
		start_time = rospy.get_time()
		# self.update_scan = False
		
		# Run a check to see if the new cloud is at least x% as big as the prev one
		# If not it is assumed that the new cloud is only a partial scan
		# if cloud.width >= 0.25 * self.prev_cloud_.width:
		self._scan_from_cloud.header = planar_cloud.header
		self._scan_from_cloud.angle_min = round(-3.141592653589793, self.res)
		self._scan_from_cloud.angle_max = round(3.141592653589793 , self.res)
		self._scan_from_cloud.angle_increment = 10**-self.res
		num_of_pts = int((round(self._scan_from_cloud.angle_max, self.res) - round(self._scan_from_cloud.angle_min, self.res)) / round(self._scan_from_cloud.angle_increment,self.res))
		self._scan_from_cloud.time_increment = 0 # I dont know
		self._scan_from_cloud.scan_time = 0 # I dont know
		self._scan_from_cloud.range_min = 0.01
		self._scan_from_cloud.range_max = 30.0
		self._scan_from_cloud.ranges = [numpy.inf] * num_of_pts
		self._scan_from_cloud.intensities = [0] * num_of_pts

		# If the polygon and the points being read are in different frames we'll transform the polygon
		if self._current_poly.header.frame_id != planar_cloud.header.frame_id:
			# rospy.loginfo("Tranforming the polygon")
			# self._tf_listener.waitForTransform(point.header.frame_id, polygon.header.frame_id, rospy.Time(), rospy.Duration(0.10))
			i = 0
			temp_poly_point = PointStamped()
			for v in self._current_poly.polygon.points:
				temp_poly_point.header = self._current_poly.header
				temp_poly_point.point = v # self._current_poly.polygon.points[i]
				temp_poly_point = self._tf_listener.transformPoint(scan.header.frame_id, temp_poly_point)
				self._current_poly.polygon.points[i] = temp_poly_point.point
				i = i + 1
			self._current_poly.header.frame_id = planar_cloud.header.frame_id

		# If the robot_height hasn't been published yet we will warn that the default is being used
		if not self._robot_height_init: rospy.logwarn('The actual robot height has not been initialized.  Currently working off of the robot height default which is set to %f', self.robot_height_default)

		self.height_sifter(planar_cloud)

		# Must sort the list of all the points by angles so we can place them in the laserscan data in the right order
		self._ang_dis_list.sort()

		i = 0
		for tup in self._ang_dis_list:
			# This is a check for at least a certain range. Basically can be used in place of the polygon filter
			if tup[0] in self._angles_checked and tup[2] < self._iteration_stamp: # If a certain object angle has been recently checked and the obstacle data is old remove it
				popped = self._ang_dis_list.pop(i)
			elif tup[1] > self.min_obj_range:
				placement_spot = int((round(self._scan_from_cloud.angle_max, self.res) + tup[0]) / round(self._scan_from_cloud.angle_increment, self.res)) - 1
				self._scan_from_cloud.ranges[placement_spot] = tup[1]
			i += 1

		# Running a check to see if the new scan data is about the same size as previous.
		# If it is less than x% of the last scan than we will assume it is not a new full scan and not publish
		# if (len(self._scan_from_cloud.ranges) - self._scan_from_cloud.ranges.count(numpy.inf)) >= 0.25 * (len(self._scan_to_publish.ranges) - self._scan_to_publish.ranges.count(numpy.inf)):
		self._scan_to_publish = self._scan_from_cloud
		self._pub.publish(self._scan_to_publish)

		# self.prev_cloud_ = cloud
		rospy.loginfo("Made it through the cloud in %f seconds", rospy.get_time() - start_time)
		self._iteration_stamp += 1

	def height_sifter(self, planar_cloud):
		"""This is the simple method of building a map of each point that interects with the robot height"""
		cloud_point = PointStamped()
		cloud_point.header = planar_cloud.header
		self._angles_checked = list()

		for p in pc2.read_points(planar_cloud, field_names = ("x", "y", "z"), skip_nans=True):
			# cloud_point.point.z = p[2]
			if p[2] < self._current_robot_height.z:
				self._angles_checked.append(round(math.atan2(p[1], p[0]), self.res))
				if not (p[2] > -self.floor_range and p[2] < self.floor_range):
					if self._poly_init:
						cloud_point.point.x = p[0]
						cloud_point.point.y = p[1]

						# Check to see if the point is within the robot polygon
						in_poly = point_in_poly(self._current_poly, cloud_point)

						# If the point is not in the polygon then we build it in as an obstacle
						if not in_poly: self._ang_dis_list.append((round(math.atan2(p[1], p[0]), self.res) , round(math.sqrt(p[0]*p[0] + p[1]*p[1]), self.res) , self._iteration_stamp))
					else:
						self._ang_dis_list.append((round(math.atan2(p[1], p[0]), self.res) , round(math.sqrt(p[0]*p[0] + p[1]*p[1]), self.res) , self._iteration_stamp))

	def slope_sifter(self, planar_cloud):
		"""TODO"""
		cloud_point = PointStamped()
		cloud_point.header = planar_cloud.header
		temp_ang_dis_hgt_list = list()

		# This is where the magic happens.  Iterating through each point in the cloud and filtering first for robot height and floor range.  Then saving if not filtered out.
		for p in pc2.read_points(planar_cloud, field_names = ("x", "y", "z"), skip_nans=True):
			# print("x", p[0], "y", p[1], "z", p[2])
			# cloud_point.point.z = p[2]
			if p[2] < self._current_robot_height.z:
				temp_ang_dis_hgt_list.append((round(math.atan2(p[1], p[0]), 1) , round(math.sqrt(p[0]*p[0] + p[1]*p[1]), self.res), p[2]))
				# print((round(math.atan2(p[1], p[0]), 1)))
		temp_ang_dis_hgt_list.sort()
		for i in range(len(temp_ang_dis_hgt_list)-1):
			if temp_ang_dis_hgt_list[i][0] < temp_ang_dis_hgt_list[i+1][0] - 2.0: # The 2 is the threshold value to determine if the next point in the cloud is in the front of the robot.  Ideally it would be PI but things arent ever perfect
			# When doing slope calc make sure the denominator isnt 0
				if not temp_ang_dis_hgt_list[i+1][1] - temp_ang_dis_hgt_list[i][1] < 0.01:
					slope = (temp_ang_dis_hgt_list[i+1][2] - temp_ang_dis_hgt_list[i][2]) / (temp_ang_dis_hgt_list[i+1][1] - temp_ang_dis_hgt_list[i][1])

		if self._poly_init:
			cloud_point.point.x = p[0]
			cloud_point.point.y = p[1]
			# Check to see if the point is within the robot polygon
			in_poly = point_in_poly(self._current_poly, cloud_point)
			# If the point is not in the polygon then we build it in as an obstacle
			if not in_poly: self._ang_dis_list.append((round(math.atan2(p[1], p[0]), self.res) , round(math.sqrt(p[0]*p[0] + p[1]*p[1]), self.res)))
		else:
			self._ang_dis_list.append((round(math.atan2(p[1], p[0]), self.res) , round(math.sqrt(p[0]*p[0] + p[1]*p[1]), self.res)))

	def update_poly(self, poly_stamped):
		"""Only updates the _current_poly to the newly published polygon"""
		self._current_poly = poly_stamped
		self._poly_init = True

	def update_height(self, height):
		"""Only updates the _current_poly to the newly published polygon"""
		self._current_robot_height = height
		self._robot_height_init = True

if __name__=='__main__':
	rospy.init_node('live_nav')

	try:
		_LiveNav = LiveNav()

		p_rate = rospy.Rate(_LiveNav.pub_rate)
		# This is the ros spinner
		while not rospy.is_shutdown():
			p_rate.sleep()

	except rospy.ROSInterruptException: 
		pass
