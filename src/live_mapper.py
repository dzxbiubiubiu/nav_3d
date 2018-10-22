#!/usr/bin/env python

#///////////////////////////////////////////////////////////////////////////////
#//      Title     : live_mapper.py
#//      Project   : nav_3d
#//      Created   : 1/23/2018
#//      Author    : Chris Suarez
#//      Platforms : Ubuntu 64-bit
#//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
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
import math
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from point_in_poly import point_in_poly


class LiveMapper:
	"""This is designed to generate a dynamic occupancy map of obstacles given XYZ point cloud inputs"""

	def __init__(self):
		# Adjustable params from live_mapper.yaml
		self.map_pub_topic = rospy.get_param("nav_3d/live_mapper/map_pub_topic", "nav_3d/live_map")
		self.planar_cloud_topic = rospy.get_param("nav_3d/live_mapper/planar_cloud_topic", "laser_stitcher/planar_cloud")
		self.poly_sub_topic = rospy.get_param("nav_3d/live_mapper/poly_topic", "nav_3d/robot_footprint")
		self.height_sub_topic = rospy.get_param("nav_3d/live_mapper/height_topic", "nav_3d/robot_height")
		self.robot_base_frame = rospy.get_param("nav_3d/live_mapper/robot_base_frame", "base_footprint")
		self.alg_name = rospy.get_param("nav_3d/live_mapper/obstacle_algorithm", "slope")
		self.map_reg = rospy.get_param("nav_3d/live_mapper/map_registration", "map")
		self.robot_height_default = rospy.get_param("nav_3d/live_mapper/robot_height_default", 1.0)
		self.floor_range = rospy.get_param("nav_3d/live_mapper/floor_range", 0.15)
		self.min_obj_dist = rospy.get_param("nav_3d/live_mapper/min_obj_dist", 0.0)
		self.max_robot_reach = rospy.get_param("nav_3d/live_mapper/max_robot_reach", 1.0)
		self.scan_res = rospy.get_param("nav_3d/live_mapper/scan_res", 3)
		self.map_res = rospy.get_param("nav_3d/live_mapper/map_res", 0.05)
		self.slope_threshold = rospy.get_param("nav_3d/live_mapper/slope_threshold", 3)
		self.drivable_height = rospy.get_param("nav_3d/live_mapper/drivable_height", 0.05)
		self.stale_map_time = rospy.get_param("nav_3d/live_mapper/stale_map_time", 10)
		self.obs_decay_time = rospy.get_param("nav_3d/live_mapper/obstacle_decay/time", 90)
		self.obs_decay_type = rospy.get_param("nav_3d/live_mapper/obstacle_decay/type", "exponential")
		self.loop_rate = rospy.get_param("nav_3d/live_mapper/loop_rate", 100)

		if not (self.obs_decay_type == "exponential" or self.obs_decay_type == "exp" or self.obs_decay_type == "Exponential" or self.obs_decay_type == "EXP" or self.obs_decay_type == "linear" or self.obs_decay_type == "lin" or self.obs_decay_type == "Linear" or self.obs_decay_type == "LIN"):
			rospy.logerr("[Nav_3d] Did not receive a valid obstacle decay type.  All obstacles will accumulate overtime and will never be removed from the map.")

		# Protected class varaibles
		self._map_to_publish = OccupancyGrid()
		self._scan_to_publish = LaserScan()
		self._current_poly = PolygonStamped()
		self._current_robot_height = Point32()
		self._robot_base_point = PointStamped()
		self._robot_base_point.header.frame_id = self.robot_base_frame
		self._occupied_list = list()
		self._poly_init = False
		self._robot_height_init = False
		self._map_init = False

		# Initialize to the default robot height
		self._current_robot_height.z = self.robot_height_default
		self._max_x = 0
		self._max_y = 0
		self._min_x = 0
		self._min_y = 0
		self._prev_map_build_time = float()


		# Publishers, subscribers, and tf listener
		rospy.Subscriber(self.planar_cloud_topic, PointCloud2, self.map_publisher)
		rospy.Subscriber(self.poly_sub_topic, PolygonStamped, self.update_poly)
		rospy.Subscriber(self.height_sub_topic, Point32, self.update_height)
		self._tf_listener = tf.TransformListener()
		
		if self.map_reg == "map" or self.map_reg == "Map" or self.map_reg == "MAP":
			self._pub = rospy.Publisher(self.map_pub_topic, OccupancyGrid, queue_size=1)
		elif self.map_reg == "scan" or self.map_reg == "Scan" or self.map_reg == "SCAN":
			self._pub = rospy.Publisher(self.map_pub_topic, LaserScan, queue_size=1)
		else:
			rospy.logerr("[Nav_3d] Live mapper failed to receive the map registration param.  Aborting live mapper.")

		rospy.loginfo("[Nav_3d] LiveMapper.py initialized")

	def map_publisher(self, planar_cloud):
		start_time = rospy.get_time()
		
		if self.map_reg == "map" or self.map_reg == "Map" or self.map_reg == "MAP":
			self._map_to_publish.header = planar_cloud.header
			if not self._map_init:
				self._map_to_publish.info.resolution = self.map_res
				self._map_to_publish.info.origin.position.z = 0		
				self._map_to_publish.info.origin.orientation.x = 0
				self._map_to_publish.info.origin.orientation.y = 0
				self._map_to_publish.info.origin.orientation.z = 0
				self._map_to_publish.info.origin.orientation.w = 1.0
				self._map_init = True
		elif self.map_reg == "scan" or self.map_reg == "Scan" or self.map_reg == "SCAN":
			self._scan_to_publish.header = planar_cloud.header
			if not self._map_init:
				self._scan_to_publish.header = planar_cloud.header
				self._scan_to_publish.angle_min = round(-3.141592653589793, self.scan_res)
				self._scan_to_publish.angle_max = round(3.141592653589793 , self.scan_res)
				self._scan_to_publish.angle_increment = 10**-self.scan_res
				num_of_pts = int((round(self._scan_to_publish.angle_max, self.scan_res) - round(self._scan_to_publish.angle_min, self.scan_res)) / round(self._scan_to_publish.angle_increment,self.scan_res))
				self._scan_to_publish.time_increment = 0 # I dont know
				self._scan_to_publish.scan_time = 0 # I dont know
				self._scan_to_publish.range_min = 0.01
				self._scan_to_publish.range_max = 30.0
				self._scan_to_publish.ranges = [numpy.inf] * num_of_pts
				self._scan_to_publish.intensities = [0] * num_of_pts
				self._map_init = True


		# If the polygon and the points being read are in different frames we'll transform the polygon
		if self._current_poly.header.frame_id != planar_cloud.header.frame_id:
			i = 0
			temp_poly_point = PointStamped()
			for v in self._current_poly.polygon.points:
				temp_poly_point.header = self._current_poly.header
				temp_poly_point.point = v # self._current_poly.polygon.points[i]
				temp_poly_point = self._tf_listener.transformPoint(planar_cloud.header.frame_id, temp_poly_point)
				self._current_poly.polygon.points[i] = temp_poly_point.point
				i += 1
			self._current_poly.header.frame_id = planar_cloud.header.frame_id

		# If the robot_height hasn't been published yet we will warn that the default is being used
		
		if not self._robot_height_init: rospy.logwarn_throttle(30, 'The actual robot height has not been initialized.  Currently working off of the robot height default which is set to %f' %self.robot_height_default)

		if self.alg_name == "height" or self.alg_name == "Height" or self.alg_name == "HEIGHT" or self.alg_name == "height_method" or self.alg_name == "height method" or self.alg_name == "Height Method" or self.alg_name == "HEIGHT METHOD": 
			self.height_method(planar_cloud)
		elif self.alg_name == "slope" or self.alg_name == "Slope" or self.alg_name == "SLOPE" or self.alg_name == "slope_method" or self.alg_name == "slope method" or self.alg_name == "Slope Method" or self.alg_name == "SLOPE METHOD": 
			self._robot_base_point = self._tf_listener.transformPoint(planar_cloud.header.frame_id, self._robot_base_point)
			self.slope_method(planar_cloud)
		else:
			rospy.logerr("[Nav_3d] Failed to receive the name of an obstacle detection algorithm to run.  Nav_3d not operational.")

		if self.map_reg == "map" or self.map_reg == "Map" or self.map_reg == "MAP":
			self._pub.publish(self._map_to_publish)
		elif self.map_reg == "scan" or self.map_reg == "Scan" or self.map_reg == "SCAN":
			self._pub.publish(self._scan_to_publish)

		# rospy.loginfo("Made it through the cloud in %f seconds", rospy.get_time() - start_time)

	def height_method(self, planar_cloud):
		"""This is the simple method of building a map of each point that interects with the robot height"""
		cloud_point = PointStamped()
		cloud_point.header = planar_cloud.header
		new_obs = list()
		time_stamp = rospy.get_time()

		# Parcing the point cloud and inputing the data into a list as XYZDT
		temp_cloud = list((p[0],p[1],p[2],math.sqrt(p[0]**2 + p[1]**2), time_stamp) for p in pc2.read_points(planar_cloud, field_names = ("x", "y", "z"), skip_nans=True)) # The other field not used here is instensity but probably could be customized

		# need to find the bounds of the map so we'll use the max values of x and y
		for p in temp_cloud:
			if p[2] < self._current_robot_height.z:
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
						if not in_poly: 
							self._occupied_list.append(p)
							new_obs.append(p)
					else:
						self._occupied_list.append(p)
						new_obs.append(p)

		if self.map_reg == "map" or self.map_reg == "Map" or self.map_reg == "MAP":
			self.map_builder(new_obs)
		elif self.map_reg == "scan" or self.map_reg == "Scan" or self.map_reg == "SCAN":
			self.scan_builder()		


	def slope_method(self, planar_cloud):
		cloud_point = PointStamped()
		cloud_point.header = planar_cloud.header
		new_obs = list()
		time_stamp = rospy.get_time()

		# Parcing the point cloud and inputing the data into a list as XYZDT
		temp_cloud = list((p[0],p[1],p[2]) for p in pc2.read_points(planar_cloud, field_names = ("x", "y", "z"), skip_nans=True)) # The other field not used here is instensity but probably could be customized

		# If the size of the cloud is odd we will throw out the middle point
		if len(temp_cloud) % 2 != 0:	
			mid_index = (len(temp_cloud) - 1) / 2
			temp_cloud.pop(mid_index) # If the cloud size is odd we will remove the middle point
		# If it is even then we will just capture the mid_index for later use
		else:
			mid_index = len(temp_cloud) / 2

		# Building the front and back halves of the cloud where
		# cloud[i][0] = x, cloud[i][1] = y, cloud[i][2] = z, cloud[i][3] = distance, cloud[i][4] = time_stamp
		front_cloud = list()
		back_cloud = list()
		for p in temp_cloud[0:mid_index]:
			front_cloud.append((p[0] - self._robot_base_point.point.x, p[1] - self._robot_base_point.point.y, p[2] - self._robot_base_point.point.z, math.sqrt((p[0]-self._robot_base_point.point.x)**2 + (p[1]-self._robot_base_point.point.y)**2), time_stamp))
		for p in temp_cloud[mid_index:]:
			back_cloud.append((p[0] - self._robot_base_point.point.x, p[1] - self._robot_base_point.point.y, p[2] - self._robot_base_point.point.z, math.sqrt((p[0]-self._robot_base_point.point.x)**2 + (p[1]-self._robot_base_point.point.y)**2), time_stamp))

		# Adding the zeroed robot base point into the cloud to be the first ref point
		front_cloud.append((0,0,0,0,time_stamp))
		back_cloud.append((0,0,0,0,time_stamp))

		# Sorting each cloud by planar distances.  This is critical for the slope method.
		front_cloud.sort(key=lambda p: p[3])
		back_cloud.sort(key=lambda p: p[3])

		# need to find the bounds of the map so we'll use the max values of x and y
		i = 1 # current point index
		g = 0 # ground point index
		for p in front_cloud[1:]:
			# During this for loop p is the same as front_cloud[i]
			if p[2] < self._current_robot_height.z and p[3] > self.min_obj_dist:
				if p[0] > self._max_x: self._max_x = p[0]
				if p[1] > self._max_y: self._max_y = p[1]
				if p[0] < self._min_x: self._min_x = p[0]
				if p[1] < self._min_y: self._min_y = p[1]

				# Sorting what is ground and what is not via slope method
				distance = (p[3] - front_cloud[g][3])
				if distance > 0: 
					slope = (p[2] - front_cloud[g][2]) / distance
				else:
					slope = numpy.inf

				if abs(slope) > self.slope_threshold:
					if self._poly_init:
						cloud_point.point.x = p[0]
						cloud_point.point.y = p[1]
					
						# Check to see if the point is within the robot polygon
						# This is made efficient by first checking to make sure the point is within the robot possible max reach
						if front_cloud[i][3] < self.max_robot_reach:
							in_poly = point_in_poly(self._current_poly, cloud_point)
					
							""" Once the point is flagged by exceeding the slope threshold we perform two more checks to ensure it is an obstacle point:
							First we'll see if it is inside the robot footprint polygon. If it is we'll assume it is a part of the robot
							Second we'll check to make that the height difference is not drivable.  It could be flagged via the slope method if it just happens
							to be a point that is very close to the previous point and is a bit offset in height via noise. """
							if not in_poly and abs(p[2] - front_cloud[g][2]) > self.drivable_height:
								self._occupied_list.append(p)
								new_obs.append(p)
					
					else:
						if abs(p[2] - front_cloud[g][2]) > self.drivable_height:
							self._occupied_list.append(p)
							new_obs.append(p)
				else:
					# The new furthest ground point is at the current index
					g = i 
			i += 1

		i = 1 # current point index
		g = 0 # ground point index
		for p in back_cloud[1:]:
			# During this for loop p is the same as back_cloud[i]
			if p[2] < self._current_robot_height.z and p[3] > self.min_obj_dist:
				if p[0] > self._max_x: self._max_x = p[0]
				if p[1] > self._max_y: self._max_y = p[1]
				if p[0] < self._min_x: self._min_x = p[0]
				if p[1] < self._min_y: self._min_y = p[1]

				# Sorting what is ground and what is not via slope method
				slope = (p[2] - back_cloud[g][2]) / (p[3] - back_cloud[g][3])

				if abs(slope) > self.slope_threshold:
					if self._poly_init:
						cloud_point.point.x = p[0]
						cloud_point.point.y = p[1]
					
						# Check to see if the point is within the robot polygon
						# This is made efficient by first checking to make sure the point is within the robot possible max reach
						if front_cloud[i][3] < self.max_robot_reach:
							in_poly = point_in_poly(self._current_poly, cloud_point)
						
							""" Once the point is flagged by exceeding the slope threshold we perform two more checks to ensure it is an obstacle point:
							First we'll see if it is inside the robot footprint polygon. If it is we'll assume it is a part of the robot
							Second we'll check to make that the height difference is not drivable.  It could be flagged via the slope method if it just happens
							to be a point that is very close to the previous point and is a bit offset in height via noise. """
							if not in_poly and abs(p[2] - back_cloud[g][2]) > self.drivable_height:
								self._occupied_list.append(p)
								new_obs.append(p)
					
					else:
						if abs(p[2] - back_cloud[g][2]) > self.drivable_height:
							self._occupied_list.append(p)
							new_obs.append(p)
				else:
					# The new furthest ground point is at the current index
					g = i 
			i += 1

		if self.map_reg == "map" or self.map_reg == "Map" or self.map_reg == "MAP":
			self.map_builder(new_obs)
		elif self.map_reg == "scan" or self.map_reg == "Scan" or self.map_reg == "SCAN":
			self.scan_builder()

	def map_builder(self, new_obs):
		"""This function builds the map based on the obstacle list provided"""
		stale_map = True if rospy.get_time() - self._prev_map_build_time > self.stale_map_time else False

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
		
		# Finding where to place each obstacle in the cell map
		if map_dim_change or stale_map:
			i = 0
			for obs in self._occupied_list:
				if self.obs_decay_type == "exponential" or self.obs_decay_type == "exp" or self.obs_decay_type == "Exponential" or self.obs_decay_type == "EXP":
					# Modeled like a decay function N0 * e ^ -t / Tau where Tau (obs_decay_time) is half life
					obs_prob = min(int(99 * math.exp(-(rospy.get_time() - obs[4]) / self.obs_decay_time)), 99) 
				elif self.obs_decay_type == "linear" or self.obs_decay_type == "lin" or self.obs_decay_type == "Linear" or self.obs_decay_type == "LIN":
					# Modeled linearly where obs_decay_time is the time for the obstacle to reach 0 probability
					obs_prob = min(int(99 - 99 * (rospy.get_time() - obs[2]) / self.obs_decay_time), 99)

				# If the probability of an obstacle is greater than 5% we will build it in the map if not let's dump it from the list
				if obs_prob > 5:	
				# If the map dimensions changed then we need to replace all obstacles
					if not (obs[0] < self._min_x or obs[0] > self._max_x or obs[1] < self._min_y or obs[1] > self._max_y):
						y_placement = int(((obs[1] - self._map_to_publish.info.origin.position.y) / self.map_res))
						x_placement = int(((obs[0] - self._map_to_publish.info.origin.position.x) / self.map_res))
						placement_spot = (y_placement) * self._map_to_publish.info.width + x_placement

					if placement_spot < len(self._map_to_publish.data): self._map_to_publish.data[placement_spot] = obs_prob
				else:
					self._occupied_list.pop(i)
				i += 1

		# If map dimensions did not change we can be smart and only place the obstacles that are new
		else:
			for obs in new_obs:
				if not (obs[0] < self._min_x or obs[0] > self._max_x or obs[1] < self._min_y or obs[1] > self._max_y):
					y_placement = int(((obs[1] - self._map_to_publish.info.origin.position.y) / self.map_res))
					x_placement = int(((obs[0] - self._map_to_publish.info.origin.position.x) / self.map_res))
					placement_spot = (y_placement) * self._map_to_publish.info.width + x_placement
					
					if placement_spot < len(self._map_to_publish.data): self._map_to_publish.data[placement_spot] = 99 # If it is a new obstacle we will give it a high probability

		self._prev_map_build_time = rospy.get_time()

	def scan_builder(self):

		i = 0
		for obs in self._occupied_list:
			if self.obs_decay_type == "exponential" or self.obs_decay_type == "exp" or self.obs_decay_type == "Exponential" or self.obs_decay_type == "EXP":
				# Modeled like a decay function N0 * e ^ -t / Tau where Tau (obs_decay_time) is half life
				obs_prob = min(int(99 * math.exp(-(rospy.get_time() - obs[4]) / self.obs_decay_time)), 99) 
			elif self.obs_decay_type == "linear" or self.obs_decay_type == "lin" or self.obs_decay_type == "Linear" or self.obs_decay_type == "LIN":
				# Modeled linearly where obs_decay_time is the time for the obstacle to reach 0 probability
				obs_prob = min(int(99 - 99 * (rospy.get_time() - obs[2]) / self.obs_decay_time), 99)
			
			# If the probability of an obstacle is greater than 5% we will build it in the map if not let's dump it from the list
			if obs_prob > 5:	
			# If the map dimensions changed then we need to replace all obstacles
				if not (obs[0] < self._min_x or obs[0] > self._max_x or obs[1] < self._min_y or obs[1] > self._max_y):
					placement_spot = int((round(self._scan_to_publish.angle_max, self.scan_res) + round(math.atan2(obs[1], obs[0]), self.scan_res)) / round(self._scan_to_publish.angle_increment, self.scan_res)) - 1
					if placement_spot < len(self._scan_to_publish.ranges): 
						self._scan_to_publish.ranges[placement_spot] = obs[3]
			else:
				self._occupied_list.pop(i)
			i += 1

	def update_poly(self, poly_stamped):
		"""Only updates the _current_poly to the newly published polygon"""
		self._current_poly = poly_stamped
		self._poly_init = True

	def update_height(self, height):
		"""Only updates the _current_poly to the newly published polygon"""
		self._current_robot_height = height
		self._robot_height_init = True

if __name__=='__main__':
	rospy.init_node('live_mapper')

	try:
		_LiveMapper = LiveMapper()

		l_rate = rospy.Rate(_LiveMapper.loop_rate)
		# This is the ros spinner
		while not rospy.is_shutdown():
			l_rate.sleep()

	except rospy.ROSInterruptException: 
		pass
