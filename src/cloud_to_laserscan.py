#!/usr/bin/env python
__author__ = "Chris Suarez"
__copyright__ = "Copyright 2018, The University of Texas at Austin, " \
                "Nuclear Robotics Group"
__credits__ = "Chris Suarez"
__license__ = "BSD"
__version__ = "0.0.8"
__maintainer__ = "Chris Suarez"
__email__ = "chriswsuarez@utexas.edu"
__status__ = "Production"
__doc__ = """This subcribes to some point cloud topic and then transforms it into a laserscan.
	It is intended for use on a mobie robot with cloud scan of the environment to use for 3D navigation.
	Any point in the cloud above the robot_height will be thrown out as well as any point that has a
	z value of 0.0 +-floor_range (assumed to be the floor that the robot is driving on)."""

import rospy
import math
import numpy
import sensor_msgs.point_cloud2 as pc2
from point_in_poly import point_in_poly
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped


class CloudToLaserscan:
	"""This subcribes to some point cloud topic and then transforms it into a laserscan.
	It is intended for use on a mobie robot with cloud scan of the environment to use for 3D navigation.
	Any point in the cloud above the robot_height will be thrown out as well as any point that has a
	z value of 0.0 +-floor_range (assumed to be the floor that the robot is driving on)."""
	
	# Change these variables to adjust the filter
	scan_sub_topic = "/laser_stitcher/nav_cloud"
	poly_sub_topic =   "/move_base/local_costmap/footprint" # "test_poly_node/polygon"
	height_sub_topic = "/nav_3d/robot_height"
	pub_topic = "/nav_3d/scan360"
	robot_height_default = 1.0 # m
	floor_range = 0.10 # m
	res = 5 # decimal points
	min_obj_range = 0.5 # Min obstacle range to be built into the costmap
	pub_rate = 4 # Hz
	
	#----------------------------------------------
	# Protected class varaibles
	_scan_to_publish = LaserScan()
	_scan_from_cloud = LaserScan()

	_current_poly = PolygonStamped()
	_poly_init = False

	_current_robot_height = Point32()
	# Initialize to the default robot height
	_current_robot_height.z = robot_height_default
	_robot_height_init = False

	def __init__(self):
		self._send_msg = False

		rospy.Subscriber(self.scan_sub_topic, PointCloud2, self.scan_builder)
		rospy.Subscriber(self.poly_sub_topic, PolygonStamped, self.update_poly)
		rospy.Subscriber(self.height_sub_topic, Point32, self.update_height)
		pub = rospy.Publisher(self.pub_topic, LaserScan, queue_size=1)

		p_rate = rospy.Rate(self.pub_rate)
		while not rospy.is_shutdown():
			if self._send_msg:
				pub.publish(self._scan_to_publish)
				self._send_msg = False
			p_rate.sleep()

	def scan_builder(self, cloud):
		start_time = rospy.get_time()
		# self.update_scan = False
		cloud_point = PointStamped()
		cloud_point.header = cloud.header
		ang_dis_list = []
		
		# Run a check to see if the new cloud is at least x% as big as the prev one
		# If not it is assumed that the new cloud is only a partial scan
		# if cloud.width >= 0.25 * self.prev_cloud_.width:
		self._scan_from_cloud.header = cloud.header
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
		if self._current_poly.header.frame_id != cloud.header.frame_id:
			# rospy.loginfo("Tranforming the polygon")
			# tf_listener.waitForTransform(point.header.frame_id, polygon.header.frame_id, rospy.Time(), rospy.Duration(0.10))
			i = 0
			temp_poly_point = PointStamped()
			for v in self._current_poly.polygon.points:
				temp_poly_point.header = self._current_poly.header
				temp_poly_point.point = v # self._current_poly.polygon.points[i]
				temp_poly_point = tf_listener.transformPoint(cloud.header.frame_id, temp_poly_point)
				self._current_poly.polygon.points[i] = temp_poly_point.point
				i = i + 1
			self._current_poly.header.frame_id = cloud.header.frame_id

		# If the robot_height hasn't been published yet we will warn that the default is being used
		if not self._robot_height_init: rospy.logwarn('The actual robot height has not been initialized.  Currently working off of the robot height default which is set to %f', self.robot_height_default)

		# This is where the magic happens.  Iterating through each point in the cloud and filtering first for robot height and floor range.  Then saving if not filtered out.
		for p in pc2.read_points(cloud, field_names = ("x", "y", "z"), skip_nans=True):
			# cloud_point.point.z = p[2]
			if p[2] < self._current_robot_height.z and not (p[2] > -self.floor_range and p[2] < self.floor_range):
				if self._poly_init:
					cloud_point.point.x = p[0]
					cloud_point.point.y = p[1]

					# Check to see if the point is within the robot polygon
					in_poly = point_in_poly(self._current_poly, cloud_point)

					# If the point is not in the polygon then we build it in as an obstacle
					if not in_poly: ang_dis_list.append( (round(math.atan2(p[1], p[0]), self.res) , round(math.sqrt(p[0]*p[0] + p[1]*p[1]), self.res) ) )

		# Must sort the list of all the points by angles so we can place them in the laserscan data in the right order
		ang_dis_list.sort()

		for tup in ang_dis_list:
			# This is a check for at least a certain range. Basically can be used in place of the polygon filter
			if tup[1] > self.min_obj_range:
				placement_spot = int((round(self._scan_from_cloud.angle_max, self.res) + tup[0]) / round(self._scan_from_cloud.angle_increment, self.res)) - 1
				self._scan_from_cloud.ranges[placement_spot] = tup[1]

		# Running a check to see if the new scan data is about the same size as previous.
		# If it is less than x% of the last scan than we will assume it is not a new full scan and not publish
		# if (len(self._scan_from_cloud.ranges) - self._scan_from_cloud.ranges.count(numpy.inf)) >= 0.25 * (len(self._scan_to_publish.ranges) - self._scan_to_publish.ranges.count(numpy.inf)):
		self._scan_to_publish = self._scan_from_cloud
		self._send_msg = True

		# self.prev_cloud_ = cloud
		rospy.loginfo("Made it through the cloud in %f seconds", rospy.get_time() - start_time)

	def update_poly(self, poly_stamped):
		"""Only updates the _current_poly to the newly published polygon"""
		self._current_poly = poly_stamped
		self._poly_init = True

	def update_height(self, height):
		"""Only updates the _current_poly to the newly published polygon"""
		self._current_robot_height = height
		self._robot_height_init = True

if __name__=='__main__':
	rospy.init_node('nav_3d/cloud_to_laserscan')
	try:
		_CloudToLaserscan = CloudToLaserscan()
	except rospy.ROSInterruptException: 
		pass
