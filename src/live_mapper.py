#!/usr/bin/env python

import rospy
import numpy
import tf
import math
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from point_in_poly import point_in_poly


class LiveMapper:
	"""This is designed to generate a dynamic occupancy map of obstacles given XYZ point cloud inputs"""
	
	# Change these variables to adjust the filter
	scan_sub_topic = "/laser_stitcher/planar_cloud"
	poly_sub_topic = "/move_base/local_costmap/footprint" # "test_poly_node/polygon"
	height_sub_topic = "/nav_3d/robot_height"
	pub_topic = "~live_map"
	robot_height_default = 1.0 # m
	floor_range = 0.15 # m
	res = 3 # decimal points
	min_obj_range = 0.1 # Min obstacle range to be built into the costmap
	map_res = 0.03 # m 
	buffer_obs = False # Add a point buffer or no? BUFFER FUNCTIONALITY NOT YET OPERATIONAL
	buffer_radius = 0.1 # m
	buffer_density = 5
	slope_threshold = 0.2
	pub_rate = 100 # Hz
	
	#----------------------------------------------
	# Protected class varaibles
	_map_to_publish = OccupancyGrid()

	_current_poly = PolygonStamped()
	_poly_init = False

	_current_robot_height = Point32()
	# Initialize to the default robot height
	_current_robot_height.z = robot_height_default
	_robot_height_init = False

	_occupied_list = list()
	_max_x = 0
	_max_y = 0
	_min_x = 0
	_min_y = 0
	_iteration_stamp = 0
	_map_init = False

	def __init__(self):
		self._pub = rospy.Publisher(self.pub_topic, OccupancyGrid, queue_size=1)
		self._tf_listener = tf.TransformListener()
		rospy.Subscriber(self.scan_sub_topic, PointCloud2, self.map_publisher)
		rospy.Subscriber(self.poly_sub_topic, PolygonStamped, self.update_poly)
		rospy.Subscriber(self.height_sub_topic, Point32, self.update_height)



	def map_publisher(self, planar_cloud):
		start_time = rospy.get_time()
		
		self._map_to_publish.header = planar_cloud.header
		if not self._map_init:
			self._map_to_publish.info.resolution = self.map_res
			self._map_to_publish.info.origin.position.z = 0		
			self._map_to_publish.info.origin.orientation.x = 0
			self._map_to_publish.info.origin.orientation.y = 0
			self._map_to_publish.info.origin.orientation.z = 0
			self._map_to_publish.info.origin.orientation.w = 1.0
			self._map_init = True

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
				i += 1
			self._current_poly.header.frame_id = planar_cloud.header.frame_id

		# If the robot_height hasn't been published yet we will warn that the default is being used
		
		if not self._robot_height_init: rospy.logwarn_throttle(30, 'The actual robot height has not been initialized.  Currently working off of the robot height default which is set to %f' %self.robot_height_default)

		self.slope_sifter(planar_cloud)

		self._pub.publish(self._map_to_publish)

		# rospy.loginfo("Made it through the cloud in %f seconds", rospy.get_time() - start_time)
		self._iteration_stamp += 1

	def height_sifter(self, planar_cloud):
		"""This is the simple method of building a map of each point that interects with the robot height"""
		cloud_point = PointStamped()
		cloud_point.header = planar_cloud.header
		points_checked = list()
		prev_height = self._map_to_publish.info.height
		prev_width = self._map_to_publish.info.width
		new_obs = list()
		
		# need to find the bounds of the map so we'll use the max values of x and y
		for p in pc2.read_points(planar_cloud, field_names = ("x", "y", "z"), skip_nans=True):
			if p[2] < self._current_robot_height.z:
				points_checked.append((p[0], p[1]))
				if p[0] > self._max_x: self._max_x = p[0]
				if p[1] > self._max_y: self._max_y = p[1]
				if p[0] < self._min_x: self._min_x = p[0]
				if p[1] < self._min_y: self._min_y = p[1]

				# This is how the height sifter determines what the ground is
				if not (p[2] > -self.floor_range and p[2] < self.floor_range):
					if self._poly_init:
						cloud_point.point.x = p[0]
						cloud_point.point.y = p[1]

						# Check to see if the point is within the robot polygon
						in_poly = point_in_poly(self._current_poly, cloud_point)

						# If the point is not in the polygon then we build it in as an obstacle
						if not in_poly: self._occupied_list.append((p[0], p[1], self._iteration_stamp))
						if not in_poly: new_obs.append((p[0], p[1]))
					else:
						self._occupied_list.append((p[0], p[1], self._iteration_stamp))
						new_obs.append((p[0], p[1]))

		self.map_builder(new_obs)


	def slope_sifter(self, planar_cloud):
		"""This is the simple method of building a map of each point that interects with the robot height"""
		cloud_point = PointStamped()
		cloud_point.header = planar_cloud.header
		# points_checked = list()
		new_obs = list()
		cloud_size = int()

		temp_cloud = list(pc2.read_points(planar_cloud, field_names = ("x", "y", "z"), skip_nans=True))
		cloud_size = len(temp_cloud)

		if cloud_size % 2 != 0:	temp_cloud.pop(int(cloud_size/2 - 0.5)) # If the cloud size is odd we will remove the middle point
		mid_index = len(temp_cloud)/2
		front_cloud = temp_cloud[0:mid_index]
		back_cloud = temp_cloud[mid_index:]
		back_cloud.reverse()
		front_cloud.insert(0, (0,0,0))
		back_cloud.insert(0, (0,0,0))
		# need to find the bounds of the map so we'll use the max values of x and y
		i = 1 # current point index
		g = 0 # ground point index
		# print(front_cloud[:])
		for p in front_cloud[1:]:
			if p[2] < self._current_robot_height.z:
				# points_checked.append((p[0], p[1]))
				if p[0] > self._max_x: self._max_x = p[0]
				if p[1] > self._max_y: self._max_y = p[1]
				if p[0] < self._min_x: self._min_x = p[0]
				if p[1] < self._min_y: self._min_y = p[1]

				# Sorting what is ground and what is not via slope method
				dist = math.sqrt(front_cloud[i][0]**2 + front_cloud[i][1]**2) - math.sqrt(front_cloud[g][0]**2 + front_cloud[g][1]**2)
				slope = (front_cloud[i][2] - front_cloud[g][2]) / dist
				if abs(slope) > self.slope_threshold:
					if self._poly_init:
						cloud_point.point.x = p[0]
						cloud_point.point.y = p[1]
					
						# Check to see if the point is within the robot polygon
						in_poly = point_in_poly(self._current_poly, cloud_point)
					
						# If the point is not in the polygon then we build it in as an obstacle
						if not in_poly: self._occupied_list.append((p[0], p[1], self._iteration_stamp))
						if not in_poly: new_obs.append((p[0], p[1]))
					
					else:
						# The new farthest ground point is at the current index
						g = i 

						self._occupied_list.append((p[0], p[1], self._iteration_stamp))
						new_obs.append((p[0], p[1]))
			i += 1

		i = 1 # current point index
		g = 0 # ground point index
		for p in back_cloud[1:]:
			if p[2] < self._current_robot_height.z:
				# points_checked.append((p[0], p[1]))
				if p[0] > self._max_x: self._max_x = p[0]
				if p[1] > self._max_y: self._max_y = p[1]
				if p[0] < self._min_x: self._min_x = p[0]
				if p[1] < self._min_y: self._min_y = p[1]

				# Sorting what is ground and what is not via slope method
				dist = math.sqrt(back_cloud[i][0]**2 + back_cloud[i][1]**2) - math.sqrt(back_cloud[g][0]**2 + back_cloud[g][1]**2)
				slope = (back_cloud[i][2] - back_cloud[g][2]) / dist
				if abs(slope) > self.slope_threshold:
					if self._poly_init:
						cloud_point.point.x = p[0]
						cloud_point.point.y = p[1]
					
						# Check to see if the point is within the robot polygon
						in_poly = point_in_poly(self._current_poly, cloud_point)
					
						# If the point is not in the polygon then we build it in as an obstacle
						if not in_poly: self._occupied_list.append((p[0], p[1], self._iteration_stamp))
						if not in_poly: new_obs.append((p[0], p[1]))
					
					else:
						# The new farthest ground point is at the current index
						g = i 

						self._occupied_list.append((p[0], p[1], self._iteration_stamp))
						new_obs.append((p[0], p[1]))
			i += 1

		self.map_builder(new_obs)


	def buffer_points(self, points):
		"""This function is designed to take in a 2D array points and 
		create add a radius of new points with the center of each of the
		original points.  This is useful when dealing with the footprint
		of a robot to create a better and safer robot footprint.
		Arguments to pass in:
		points = 2D array of points numpy.array([:,:])
		radius = buffer distance from the center of each point
		density = number of points to form the circle around each point
		Returns an 2D list of points"""
		buffed_occupied_list = list()
		theta_increment = 2 * 3.1416 / self.buffer_density

		for point in points:
			current_theta = 0
			for i in list(range(0, self.buffer_density)):
				for rad_i in list(range(1, self.buffer_density)):
					x_new = point[0] + self.buffer_radius * (self.buffer_density / rad_i) * numpy.cos(current_theta + theta_increment)
					y_new = point[1] + self.buffer_radius * (self.buffer_density / rad_i) * numpy.sin(current_theta + theta_increment)
					buffed_occupied_list.append((x_new, y_new))

				current_theta += theta_increment
		
		return buffed_occupied_list

	def map_builder(self, new_obs):
		"""This function builds the map based on the obstacle list provided"""

		prev_height = self._map_to_publish.info.height
		prev_width = self._map_to_publish.info.width

		# Build the map now
		self._map_to_publish.info.origin.position.x = round(self._min_x, 2)
		self._map_to_publish.info.origin.position.y = round(self._min_y, 2)
		self._map_to_publish.info.width = int((self._max_x - self._min_x) / self.map_res)
		self._map_to_publish.info.height = int((self._max_y - self._min_y) / self.map_res)
		
		# Checking to see if the size of the map has changed.  No need to refill an entire new map if it is the same size
		map_dim_change = (prev_width != self._map_to_publish.info.width or prev_height != self._map_to_publish.info.height)
		if map_dim_change:	self._map_to_publish.data = [0] * (self._map_to_publish.info.width * self._map_to_publish.info.height)

		# Creating the buffer zone for each obstacle point
		# BUFFER FUNCTIONALITY NOT YET OPERATIONAL.  BUGS MUST BE WORKED OUT
		if self.buffer_obs: self._occupied_list = self.buffer_points(self._occupied_list)
		
		# Finding where to place each obstacle in the cell map
		if map_dim_change:
			for obs in self._occupied_list:
			# If the map dimensions changed then we need to replace all obstacles
				if not (obs[0] < self._min_x or obs[0] > self._max_x or obs[1] < self._min_y or obs[1] > self._max_y):
					y_placement = int(((obs[1] - self._map_to_publish.info.origin.position.y) / self.map_res) - 1)
					x_placement = int(((obs[0] - self._map_to_publish.info.origin.position.x) / self.map_res) - 1)
					placement_spot = (y_placement - 1) * self._map_to_publish.info.width + x_placement
					self._map_to_publish.data[placement_spot] = 99

		# If map dimensions did not change we can be smart and only place the obstacles that are new
		else:
			for obs in new_obs:
				# if obs[2] < self._iteration_stamp:
					if not (obs[0] < self._min_x or obs[0] > self._max_x or obs[1] < self._min_y or obs[1] > self._max_y):
						y_placement = int(((obs[1] - self._map_to_publish.info.origin.position.y) / self.map_res) - 1)
						x_placement = int(((obs[0] - self._map_to_publish.info.origin.position.x) / self.map_res) - 1)
						placement_spot = (y_placement - 1) * self._map_to_publish.info.width + x_placement
						self._map_to_publish.data[placement_spot] = 99

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
		_LiveMapper = LiveMapper()

		p_rate = rospy.Rate(_LiveMapper.pub_rate)
		# This is the ros spinner
		while not rospy.is_shutdown():
			p_rate.sleep()

	except rospy.ROSInterruptException: 
		pass
