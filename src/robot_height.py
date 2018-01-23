#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Point32
# from std_msgs.msg import Float32
# from tf import TransformListener
import tf

class robot_height:
	""" This class is made to calculate the height of a 
	robot from base_link to the highest measured link. 
	It publishes the x,y,z coordinates of only the highest robot point"""

	# Adjustable settings
	pub_rate_ = 1 # hz
	ref_link_ = '/base_link'
	#----------------------------------------------

	highest_point_ = Point32()
	highest_link_ = 'Not init'
	send_msg_ = False

	def __init__(self):
		pub = rospy.Publisher('/robot_height', Point32, queue_size=1)

		pub_rate = rospy.Rate(self.pub_rate_)
		while not rospy.is_shutdown():
			self.find_height()
			if self.send_msg_:
				pub.publish(self.highest_point_)
			pub_rate.sleep()

	def find_height(self):

		# Get all the names for each link in the system
		links = tf_listener.getFrameStrings()

		prev_high_link = self.highest_link_
		prev_high_point = self.highest_point_.z

		# Check the position of each link and check to find the highest link
		for frame in links:
			try:
				transform = tf_listener.lookupTransform(self.ref_link_, frame, rospy.Time())

				# The position array is the first section of the tuple transform[0] = [x,y,z]
				position = transform[0]

				if position[2] > self.highest_point_.z:
					self.highest_link_ = frame
					self.highest_point_.x = position[0]
					self.highest_point_.y = position[1]
					self.highest_point_.z = position[2]
			except tf.Exception:
				rospy.logerr_throttle(5, 'TF has thrown an exception.  Will retry the TF call')

		if not prev_high_link == self.highest_link_:
			rospy.loginfo("Highest link is now: %s at height %f meters above base_link", self.highest_link_, self.highest_point_.z)
		elif prev_high_link == self.highest_link_ and not round(prev_high_point,2) == round(self.highest_point_.z,2):
			rospy.loginfo("%s is still the highest link but has moved up to: %f meters above base link", self.highest_link_, self.highest_point_.z)
		else:
			pass
		
		self.send_msg_ = True

if __name__=='__main__':
	tf_listener = tf.TransformListener()
	rospy.init_node('robot_height')
	try:
		robot_height = robot_height()
	except rospy.ROSInterruptException: 
		pass
