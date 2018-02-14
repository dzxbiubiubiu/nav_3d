#!/usr/bin/env python

import rospy
import math
import numpy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped


class cloud_to_laserscan:
	"""This subcribes to some point cloud topic and then transforms it into a laserscan.
	It is intended for use on a mobie robot with cloud scan of the environment to use for 3D navigation.
	Any point in the cloud above the robot_height will be thrown out as well as any point that has a
	z value of 0.0 +-floor_range (assumed to be the floor that the robot is driving on)."""
	
	# Change these variables to adjust the filter
	scan_sub_topic_ = "/laser_stitcher/nav_cloud"
	poly_sub_topic =   "/move_base/local_costmap/footprint" # "test_poly_node/polygon"
	height_sub_topic = "/robot_height"
	pub_topic_ = "/scan360"
	robot_height_default_ = 1.0 # m
	floor_range_ = 0.10 # m
	res_ = 5 # decimal points
	min_obj_range_ = 0.5 # Min obstacle range to be built into the costmap
	pub_rate_ = 4 # Hz
	#----------------------------------------------

	scan_to_publish_ = LaserScan()
	scan_from_cloud_ = LaserScan()
	# prev_cloud_ = PointCloud2()

	current_poly = PolygonStamped()
	poly_init = False

	current_robot_height = Point32()
	# Initialize to the default robot height
	current_robot_height.z = robot_height_default_
	robot_height_init = False

	def __init__(self):
		self.send_msg_ = False

		rospy.Subscriber(self.scan_sub_topic_, PointCloud2, self.scan_builder)
		rospy.Subscriber(self.poly_sub_topic, PolygonStamped, self.update_poly)
		rospy.Subscriber(self.height_sub_topic, Point32, self.update_height)
		pub = rospy.Publisher(self.pub_topic_, LaserScan, queue_size=1)

		p_rate = rospy.Rate(self.pub_rate_)
		while not rospy.is_shutdown():
			if self.send_msg_:
				pub.publish(self.scan_to_publish_)
				self.send_msg_ = False
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
		self.scan_from_cloud_.header = cloud.header
		self.scan_from_cloud_.angle_min = round(-3.141592653589793, self.res_)
		self.scan_from_cloud_.angle_max = round(3.141592653589793 , self.res_)
		self.scan_from_cloud_.angle_increment = 10**-self.res_
		num_of_pts = int((round(self.scan_from_cloud_.angle_max, self.res_) - round(self.scan_from_cloud_.angle_min, self.res_)) / round(self.scan_from_cloud_.angle_increment,self.res_))
		self.scan_from_cloud_.time_increment = 0 # I dont know
		self.scan_from_cloud_.scan_time = 0 # I dont know
		self.scan_from_cloud_.range_min = 0.01
		self.scan_from_cloud_.range_max = 30.0
		self.scan_from_cloud_.ranges = [numpy.inf] * num_of_pts
		self.scan_from_cloud_.intensities = [0] * num_of_pts

		# If the polygon and the points being read are in different frames we'll transform the polygon
		if self.current_poly.header.frame_id != cloud.header.frame_id:
			# rospy.loginfo("Tranforming the polygon")
			# tf_listener.waitForTransform(point.header.frame_id, polygon.header.frame_id, rospy.Time(), rospy.Duration(0.10))
			i = 0
			temp_poly_point = PointStamped()
			for v in self.current_poly.polygon.points:
				temp_poly_point.header = self.current_poly.header
				temp_poly_point.point = v # self.current_poly.polygon.points[i]
				temp_poly_point = tf_listener.transformPoint(cloud.header.frame_id, temp_poly_point)
				self.current_poly.polygon.points[i] = temp_poly_point.point
				i = i + 1
			self.current_poly.header.frame_id = cloud.header.frame_id

		# If the robot_height hasn't been published yet we will warn that the default is being used
		if not self.robot_height_init: rospy.logwarn('The actual robot height has not been initialized.  Currently working off of the robot height default which is set to %f', self.robot_height_default_)

		# This is where the magic happens.  Iterating through each point in the cloud and filtering first for robot height and floor range.  Then saving if not filtered out.
		for p in pc2.read_points(cloud, field_names = ("x", "y", "z"), skip_nans=True):
			#rospy.loginfo(" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
			# cloud_point.point.z = p[2]
			if p[2] < self.current_robot_height.z and not (p[2] > -self.floor_range_ and p[2] < self.floor_range_):
				if self.poly_init:
					cloud_point.point.x = p[0]
					cloud_point.point.y = p[1]

					# Check to see if the point is within the robot polygon
					in_poly = point_in_poly(self.current_poly, cloud_point)

					# If the point is not in the polygon then we build it in as an obstacle
					if not in_poly: ang_dis_list.append( (round(math.atan2(p[1], p[0]), self.res_) , round(math.sqrt(p[0]*p[0] + p[1]*p[1]), self.res_) ) )

		# Must sort the list of all the points by angles so we can place them in the laserscan data in the right order
		ang_dis_list.sort()

		for tup in ang_dis_list:
			# This is a check for at least a certain range. Basically can be used in place of the polygon filter
			if tup[1] > self.min_obj_range_:
				placement_spot = int((round(self.scan_from_cloud_.angle_max, self.res_) + tup[0]) / round(self.scan_from_cloud_.angle_increment, self.res_)) - 1
				self.scan_from_cloud_.ranges[placement_spot] = tup[1]

		# Running a check to see if the new scan data is about the same size as previous.
		# If it is less than x% of the last scan than we will assume it is not a new full scan and not publish
		# if (len(self.scan_from_cloud_.ranges) - self.scan_from_cloud_.ranges.count(numpy.inf)) >= 0.25 * (len(self.scan_to_publish_.ranges) - self.scan_to_publish_.ranges.count(numpy.inf)):
		self.scan_to_publish_ = self.scan_from_cloud_
		self.send_msg_ = True

		# self.prev_cloud_ = cloud
		rospy.loginfo("Made it through the cloud in %f seconds", rospy.get_time() - start_time)

	def update_poly(self, poly_stamped):
		"""Only updates the current_poly to the newly published polygon"""
		self.current_poly = poly_stamped
		self.poly_init = True

	def update_height(self, height):
		"""Only updates the current_poly to the newly published polygon"""
		self.current_robot_height = height
		self.robot_height_init = True

def point_in_poly(polygon, point):
	"""This is a method of determining if a given point is within a given polygon. 
	It assumes that the given point and polygon are both stamped instances.

	NOTE: If the point and polygon are in different frames then it wants to transform the point into the same frame.
	However, if this is being called very often the function transformPoint will slow performance greatly. 
	In a case like that, it is better to ensure the point and polgyon are in the same frame already."""
	if not isinstance(polygon, PolygonStamped):
		rospy.loginfo("A stamped polygon was not passed into point_in_poly")

	if not isinstance(point, PointStamped):
		rospy.loginfo("A stamped point was not passed into point_in_poly")

	n = len(polygon.polygon.points)

	result = False

	# Transforming the point into the same frame as the polygon if it is necessary
	if polygon.header.frame_id != point.header.frame_id:
		point = tf_listener.transformPoint(polygon.header.frame_id, point)

	# Getting rid of stamped portions
	polygon = polygon.polygon
	point = point.point

	# Point in polygon algorithm
	p1x = polygon.points[0].x
	p1y = polygon.points[0].y
	for i in range(n):
		p2x = polygon.points[i].x
		p2y = polygon.points[i].y
		# print("p1y : %f p2y : %f" %(p1y,p2y))
		if point.y > min(p1y,p2y):
			# print "Y is above min"
			if point.y <= max(p1y,p2y):
				# print "Y is below max"
				if point.x <= max(p1x,p2x):
					# print "x below max"
					if p1y != p2y:
						xinters = (point.y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
					if p1x == p2x or point.x <= xinters:
						result = not result
		p1x,p1y = p2x,p2y
	return result

if __name__=='__main__':
	rospy.init_node('cloud_to_laserscan')
	try:
		cloud_to_laserscan = cloud_to_laserscan()
	except rospy.ROSInterruptException: 
		pass
