
///////////////////////////////////////////////////////////////////////////////
//      Title     : post_mapper.h
//      Project   : nav_3d
//      Created   : 1/1/2019
//      Author    : Chris Suarez
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2018. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef POST_MAPPER_H
#define POST_MAPPER_H

#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include "point_in_poly.h"

#include <tf/transform_listener.h>

class PostMapper
{
public:
	PostMapper();
private:
	ros::NodeHandle nh_;
	ros::Subscriber cloud_sub_, poly_sub_, height_sub_, map_sub_;
	ros::Publisher map_pub_, scan_pub_, viz_pub_;
	tf::TransformListener listener_;

	// Point structure that includes a planar distance calc, time stamp, and obstacle data
	struct point_XYZDATO
	{
		float x, y, z, distance, azimuth;
		ros::Time time_stamp;
		int obstacle; // Defined point state.  0 = Not an obstacle, 1 = Obstacle, 2 = Likely Drivable (unsure), 3 = Too far away and too high/low (unsure), 4 = Height Warning
	};

	//Varibles that will be set via yaml file
	int lidar_config_, scan_res_;
	std::string alg_name_, map_reg_, obs_decay_type_, robot_base_frame_, lidar_frame_;
	float robot_height_default_, floor_range_, slope_threshold_, drivable_height_, max_step_height_, min_obj_dist_, 
		max_check_dist_, max_robot_reach_, stale_map_time_, obs_decay_time_, map_res_, obs_decay_factor_, loop_rate_,
		max_ditch_depth_, viz_timer_, robot_height_buffer_, robot_height_warning_, lidar_tilt_threshold_, recent_obs_purge_time_, parsing_segments_;
	bool viz_tool_;

	//Robot footprint variables
	geometry_msgs::PolygonStamped current_poly_;
	geometry_msgs::PointStamped robot_base_point_, robot_base_point_tfd_;
	bool poly_init_;

	//Robot height variables
	geometry_msgs::Point32 current_robot_height_;
	bool robot_height_init_;

	// Obstacle Detection Variables
	std::vector<std::vector<point_XYZDATO> > parsed_clouds_;

	//Map variables
	nav_msgs::OccupancyGrid map_to_publish_;
	sensor_msgs::LaserScan scan_to_publish_;
	std::vector<point_XYZDATO> occupied_list_, points_checked_;
	std::vector<int> recent_obs_cells_;
	ros::Time prev_recent_obs_purge_time_;
	bool map_dim_change_;

	int num_of_pts_;
	float max_x_, max_y_, min_x_, min_y_; // The farthest point measured in meters by the lidar in the x direction
	float upper_x_, upper_y_; // The upper bound in meters of the map in the x and y directions (must be greater than max_x and max_y)
	float prev_max_x_, prev_max_y_, prev_min_x_, prev_min_y_;
	float prev_origin_x_, prev_origin_y_, prev_upper_x_, prev_upper_y_;
	int prev_height_, prev_width_;
	int dx_, d_x_, dy_, d_y_;
	std::vector<int> prev_map_data_;

	ros::Time start_time_;
	ros::Time prev_map_build_time_;
	bool map_init_, scan_init_;


	// Visualization tool variables
	ros::Time prev_viz_pub_time_;
	pcl::PointCloud<pcl::PointXYZI> viz_cloud_;
	sensor_msgs::PointCloud2 viz_cloud_pub_;


	//Callbacks
	void mainCallback(const sensor_msgs::PointCloud2::ConstPtr& planar_cloud);
	void heightMethod(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&  planar_cloud);
	void slopeMethod();
	void cloudParser(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& planar_cloud);
	void mapBuilder();
	void scanBuilder();
	void mapSizeMaintainer(const pcl::PointXYZ point);
	void mapDiscretizer();
	void convertPoly(const std_msgs::Header new_header);
	void updatePoly(const geometry_msgs::PolygonStamped::ConstPtr& new_poly_stamped);
	void updateHeight(const geometry_msgs::Point32::ConstPtr& new_height);
	void updateMap(const nav_msgs::OccupancyGrid::ConstPtr& new_map);
	void visualizationTool();
	static bool compareDistance(point_XYZDATO p1, point_XYZDATO p2);
};

#endif //POST_MAPPER_H
