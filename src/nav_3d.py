#!/usr/bin/env python
__author__ = "Chris Suarez"
__copyright__ = "Copyright 2018, The University of Texas at Austin, " \
                "Nuclear Robotics Group"
__credits__ = "Chris Suarez"
__license__ = "BSD"
__version__ = "0.0.1"
__maintainer__ = "Chris Suarez"
__email__ = "chriswsuarez@utexas.edu"
__status__ = "Production"
__doc__ = """This subcribes to some point cloud topic and then transforms it into a laserscan.
	It is intended for use on a mobie robot with cloud scan of the environment to use for 3D navigation.
	Any point in the cloud above the robot_height will be thrown out as well as any point that has a
	z value of 0.0 +-floor_range (assumed to be the floor that the robot is driving on)."""

import rospy
from cloud_to_laserscan import CloudToLaserscan
from robot_state import RobotState
from live_nav import LiveNav
from live_mapper import LiveMapper

if __name__=='__main__':
	rospy.init_node('nav_3d')

	try:
		_RobotState = RobotState()
		# _CloudToLaserscan = CloudToLaserscan()
		# _LiveNav = LiveNav()
		_LiveMap = LiveMapper()

		pub_rate = max(_RobotState.pub_rate, _LiveMap.pub_rate)
		p_rate = rospy.Rate(pub_rate)
		while not rospy.is_shutdown():
			_RobotState.analyze_links()
			p_rate.sleep()

	except rospy.ROSInterruptException: 
		pass